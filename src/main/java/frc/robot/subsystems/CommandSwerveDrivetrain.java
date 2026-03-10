package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.OptionalInt;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModule.ModuleRequest;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.SwerveModuleConfig;
import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    protected final CANBus canBus;
    protected Pigeon2 gyro;
    protected SwerveDriveKinematics kinematics;
    protected SwerveDriveOdometry odometry;
    protected SwerveDrivePoseEstimator poseEstimator;
    protected Pose2d initPose = new Pose2d();
    protected Pose2d currentPose;
    protected Timer timer = new Timer();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    public Alliance allianceColor = Alliance.Blue;
    public OptionalInt stationLocation;
    
    private com.pathplanner.lib.config.RobotConfig robotConfig;
    private PIDConstants translationPid = new PIDConstants(5.0, 0.0, 0.0);
    private PIDConstants rotationPid = new PIDConstants(5.0, 0.0, 0.0);

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> {
                /* output is actually radians per second, but SysId only supports "volts" */
                setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
                /* also log the requested output for SysId */
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            this
        )
    );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants   Drivetrain-wide constants for the swerve drive
     * @param modules               Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        frc.robot.RobotConfig config,
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules)
    {
        super(drivetrainConstants, modules);

        canBus = new CANBus(config.driveCANBus);
        gyro = new Pigeon2(config.pigeonId, canBus);
        gyro.reset();

        initOdometry(
            new Translation2d(config.frontLeft.xPos, config.frontLeft.yPos),
            new Translation2d(config.frontRight.xPos, config.frontRight.yPos),
            new Translation2d(config.backLeft.xPos, config.backLeft.yPos),
            new Translation2d(config.backRight.xPos, config.backRight.yPos)
        );

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified or set to 0 Hz, this is 250 Hz on
     *                                CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        frc.robot.RobotConfig config,
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules)
    {
        super(drivetrainConstants, odometryUpdateFrequency, modules);

        canBus = new CANBus(config.driveCANBus);
        gyro = new Pigeon2(config.pigeonId, canBus);
        initOdometry(
            new Translation2d(config.frontLeft.xPos, config.frontLeft.yPos),
            new Translation2d(config.frontRight.xPos, config.frontRight.yPos),
            new Translation2d(config.backLeft.xPos, config.backLeft.yPos),
            new Translation2d(config.backRight.xPos, config.backRight.yPos)
        );

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so users should not construct
     * the devices themselves. If they need the devices, they can access them through
     * getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If
     *                                  unspecified or set to 0 Hz, this is 250 Hz on
     *                                  CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation
     *                                  in the form [x, y, theta]ᵀ, with units in meters
     *                                  and radians
     * @param modules                   Constants for each specific module
     */
    public CommandSwerveDrivetrain(
        frc.robot.RobotConfig config,
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules)
    {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);

        canBus = new CANBus(config.driveCANBus);
        gyro = new Pigeon2(config.pigeonId, canBus);
        initOdometry(
            new Translation2d(config.frontLeft.xPos, config.frontLeft.yPos),
            new Translation2d(config.frontRight.xPos, config.frontRight.yPos),
            new Translation2d(config.backLeft.xPos, config.backLeft.yPos),
            new Translation2d(config.backRight.xPos, config.backRight.yPos)
        );

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        // SwerveModuleState frontLeft = moduleStates[0];
        // SwerveModuleState frontRight = moduleStates[1];
        // SwerveModuleState backLeft = moduleStates[2];
        // SwerveModuleState backRight = moduleStates[3];

        // ModuleRequest frontLeftRequest = new ModuleRequest();
        // frontLeftRequest.withState(moduleStates[0]);
        // ModuleRequest frontLeftRequest = ModuleRequest.create()
        //     .withSteerAngle(frontLeft.angle)
        //     .withDriveVelocity(frontLeft.speed);

        //TODO Compare with setControl(SwerveRequest request)
        for (int i = 0; i < 4; ++i)
            getModule(i).apply(new ModuleRequest().withState(moduleStates[i]));
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Quasistatic test
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine
     * specified by {@link #m_sysIdRoutineToApply}.
     *
     * @param direction Direction of the SysId Dynamic test
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
//TODOVG !!!
//            System.out.println("Drive-periodic " + m_hasAppliedOperatorPerspective);

            DriverStation.getAlliance().ifPresent(color -> {
                allianceColor = color;
                stationLocation = DriverStation.getLocation();
                m_hasAppliedOperatorPerspective = true;
                //TODO
                // setOperatorPerspectiveForward(
                //     color == Alliance.Red
                //         ? kRedAlliancePerspectiveRotation
                //         : kBlueAlliancePerspectiveRotation
                // );
            });
        }

        // Get the rotation of the robot from the gyro.
        Rotation2d rotation = gyro.getRotation2d();
        // Update the pose
        currentPose = odometry.update(rotation, getState().ModulePositions);

        Pose2d pose = poseEstimator.updateWithTime(Timer.getFPGATimestamp(), rotation, getState().ModulePositions);

        SmartDashboard.putNumber("Position_X", currentPose.getX());
        SmartDashboard.putNumber("Position_Y", currentPose.getY());
        SmartDashboard.putNumber("Rotation_Grad", currentPose.getRotation().getDegrees());

        SmartDashboard.putBoolean("isDisabled_Drive", DriverStation.isDisabled());

        SmartDashboard.putNumber("Acc_X",  gyro.getAccelerationX().getValueAsDouble());
        SmartDashboard.putNumber("Acc_Y",  gyro.getAccelerationY().getValueAsDouble());
        SmartDashboard.putNumber("Acc_Z",  gyro.getAccelerationZ().getValueAsDouble());
        SmartDashboard.putNumber("Grav_X",  gyro.getGravityVectorX().getValueAsDouble());
        SmartDashboard.putNumber("Grav_Y",  gyro.getGravityVectorY().getValueAsDouble());
        SmartDashboard.putNumber("Grav_Z",  gyro.getGravityVectorZ().getValueAsDouble());
        SmartDashboard.putNumber("Magn_X",  gyro.getMagneticFieldX().getValueAsDouble());
        SmartDashboard.putNumber("Magn_Y",  gyro.getMagneticFieldY().getValueAsDouble());
        SmartDashboard.putNumber("Magn_Z",  gyro.getMagneticFieldZ().getValueAsDouble());

        SmartDashboard.putNumber("Yaw",  gyro.getYaw().getValueAsDouble());
        SmartDashboard.putNumber("Roll",  gyro.getRoll().getValueAsDouble());
        SmartDashboard.putNumber("Pitch",  gyro.getPitch().getValueAsDouble());
        SmartDashboard.putNumber("Quat_X",  gyro.getQuatX().getValueAsDouble());
        SmartDashboard.putNumber("Quat_Y",  gyro.getQuatY().getValueAsDouble());
        SmartDashboard.putNumber("Quat_Z",  gyro.getQuatZ().getValueAsDouble());
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue to apply to future measurements until a subsequent call to
     * {@link #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement
     *     in the form [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs
    ) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    protected void initOdometry(Translation2d frontLeft, Translation2d frontRight, Translation2d backLeft, Translation2d backRight) {
//        kinematics = new SwerveDriveKinematics(frontLeft, frontRight, backLeft, backRight);
        kinematics = getKinematics();

        SwerveDriveState driveState = getState();

        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(),
            driveState.ModulePositions, initPose);
    
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getRotation2d(),
            driveState.ModulePositions, initPose);
    }

    /** Get the estimated pose of the swerve drive on the field. */
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** The heading of the swerve drive's estimated pose on the field. */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /** Raw gyro yaw (this may not match the field heading!). */
    public Rotation2d getGyroYaw() {
        return gyro.getRotation2d();
    }

    /** Get the chassis speeds of the robot (vx, vy, omega) from the swerve module states. */
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public Pose2d getPosition() {
        return currentPose;  //odometry.getPoseMeters();
    }

    public Rotation2d getRotation() {
        return gyro.getRotation2d();  //odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
//        gyro.reset();
        odometry.resetPosition(gyro.getRotation2d(), getState().ModulePositions, pose);
    }

    public Rotation2d getYaw() {
//        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
        return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    //TODO
    public void drive(ChassisSpeeds targetSpeeds) {
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(targetSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.DriveConstants.maxSpeed);

        SwerveModuleState[] moduleStates = getState().ModuleStates;
        for(int i = 0; i < moduleStates.length; ++i){
            SwerveModuleState mod = moduleStates[i];
//            desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
            // mod.setAngle(desiredState);
            // mod.setSpeed(desiredState, true);
    
//            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], true);
        }
    }

    protected boolean initPathPlanner() {
        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        try{
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            //TODO :drive()
            (speeds, feedforwards) -> setChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                translationPid, rotationPid
            ),
            robotConfig, // The robot configuration
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                return allianceColor == Alliance.Red;
            },
            this // Reference to this subsystem to set requirements
        );
        return true;
    }

    public Command followPathCommand(String pathName) {
        try{
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    //           public FollowPathCommand(
    //   PathPlannerPath path,
    //   Supplier<Pose2d> poseSupplier,
    //   Supplier<ChassisSpeeds> speedsSupplier,
    //   BiConsumer<ChassisSpeeds, DriveFeedforwards> output,
    //   PathFollowingController controller,
    //   RobotConfig robotConfig,
    //   BooleanSupplier shouldFlipPath,
    //   Subsystem... requirements) {

            return new FollowPathCommand(
                path,
                this::getPose, // Robot pose supplier
                this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                //TODO :drive()
                (speeds, feedforwards) -> setChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    translationPid, rotationPid
                ),
                robotConfig, // The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    return allianceColor == Alliance.Red;
                },
                this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
}
