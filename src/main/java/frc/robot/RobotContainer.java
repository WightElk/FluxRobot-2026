// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
//import static frc.robot.generated.TunerConstants.ConstantCreator;

import java.io.IOException;
import java.util.function.Supplier;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
//import frc.robot.commands.AlignedDriveToTag;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveToTag;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  public final SwerveDrivetrainConstants DrivetrainConstants;

  // Setting up bindings for necessary control of the swerve drive platform
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
    .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  public final CommandSwerveDrivetrain drivetrain;

  private final VisionSubsystem vision;

  public AprilTagFieldLayout fieldLayout;
  public Pose3d fieldOrigin = new Pose3d();
  public double fieldLength = 0.0;
  public double fieldWidth = 0.0;

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

  // Set to 1 to use driver controller for everything, 2 to use operator controller for elevator and tray
  protected boolean useTwoControllers = OperatorConstants.UseTwoControllers;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  protected final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DriverControllerPort);

  double xStartPos = OperatorConstants.xStartPos;
  double xMiddlePos = OperatorConstants.xMiddlePos;
  double yStartPos = OperatorConstants.yStartPos;
  double yMiddlePos = OperatorConstants.yMiddlePos;
  double yMaxPos = OperatorConstants.yMaxPos;

  private final PiecewiseSensitivity sensitivityPos = 
      new PiecewiseSensitivity(OperatorConstants.xStartPos, OperatorConstants.xMiddlePos, OperatorConstants.yStartPos, OperatorConstants.yMiddlePos, OperatorConstants.yMaxPos);

  private final PiecewiseSensitivity sensitivityRot =
    new PiecewiseSensitivity(OperatorConstants.xStartRot, OperatorConstants.xMiddleRot, OperatorConstants.yStartRot, OperatorConstants.yMiddleRot, OperatorConstants.yMaxRot);

  private final Sensitivity sensitivityPos2 = 
      new Sensitivity(OperatorConstants.Threshold, OperatorConstants.CuspX, OperatorConstants.LinCoef, OperatorConstants.SpeedLimitX);

  private final Sensitivity sensitivityRot2 =
      new Sensitivity(OperatorConstants.RotThreshold, OperatorConstants.RotCuspX, OperatorConstants.RotLinCoef, OperatorConstants.SpeedLimitRot);

  protected final Telemetry logger = new Telemetry(MaxSpeed);

// /  public final CANBus kCANBus;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer(RobotConfig config, boolean useVision)
  {
//    kCANBus = new CANBus(config.driveCANBus, "./logs/example.hoot");
    DrivetrainConstants = new SwerveDrivetrainConstants()
      .withCANBusName(config.driveCANBus)
      .withPigeon2Id(config.pigeonId)
      .withPigeon2Configs(config.pigeonConfigs);

    drivetrain = createDrivetrain(config);
    drivetrain.registerTelemetry(logger::telemeterize);

    // /edu/wpi/first/apriltag/2025-reefscape-andymark.json
    //String path = Filesystem.getDeployDirectory().getPath() + AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile;
    String path = Filesystem.getDeployDirectory().getPath() + "/" +  Constants.fieldLayoutFile;
    if (useVision) {
      try {
          fieldLayout = new AprilTagFieldLayout(path);
          fieldLength = fieldLayout.getFieldLength();
          fieldWidth = fieldLayout.getFieldWidth();
          fieldOrigin = fieldLayout.getOrigin();
      } catch (IOException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
      }
    }

    // Single camera vision for AprilTag detection
    vision = useVision ? new VisionSubsystem(VisionConstants.CAMERA_NAME, VisionConstants.CameraBackName, fieldLayout, drivetrain::addVisionMeasurement) : null;

    SmartDashboard.putNumber("Start_X", xStartPos);
    SmartDashboard.putNumber("Middle_X", xMiddlePos);
    SmartDashboard.putNumber("Start_Y", yStartPos);
    SmartDashboard.putNumber("Middle_Y", yMiddlePos);
    SmartDashboard.putNumber("Max_Y", yMaxPos);

    // Configure trigger bindings
    //configureBindings();
  }

  private boolean m_hasAppliedOperatorPerspective = false;
  private Alliance allianceColor = Alliance.Blue;


  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  protected void configureBindings()
  {
    // System.out.println("configureBindings");
    // AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2023ChargedUp);
    // Pose3d origin = layout.getOrigin();
    // double length = layout.getFieldLength();
    // double width = layout.getFieldWidth();

    // System.out.println("Origin: " + origin);
    // System.out.println("Size: " + width + "x" + length);

    /*  Example: How to bind commands to triggers
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand());
    */

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
     // Drivetrain will execute this command periodically
      drivetrain.applyRequest(() -> {
        // double velX = MaxSpeed * sensitivityPos.transfer(-driverController.getLeftY());
        // double velY = MaxSpeed * sensitivityPos.transfer(-driverController.getLeftX());
        // double velRot = MaxAngularRate * sensitivityPos.transfer(-driverController.getRightX());
        double xStart = SmartDashboard.getNumber("Start_X", OperatorConstants.xStartPos);
        double xMiddle = SmartDashboard.getNumber("Middle_X", OperatorConstants.xMiddlePos);
        double yStart = SmartDashboard.getNumber("Start_Y", OperatorConstants.yStartPos);
        double yMiddle = SmartDashboard.getNumber("Middle_Y", OperatorConstants.yMiddlePos);
        double yMax = SmartDashboard.getNumber("Max_Y", OperatorConstants.yMaxPos);

        boolean changed = xStartPos != xStart;
        if (changed)
          xStartPos = xStart;
        changed = changed || xMiddlePos != xMiddle;
        if (xMiddlePos != xMiddle)
          xMiddlePos = xMiddle;
        changed = changed || yStartPos != yStart;
        if (yStartPos != yStart)
          yStartPos = yStart;
        changed = changed || yMiddlePos != yMiddle;
        if (yMiddlePos != yMiddle)
          yMiddlePos = yMiddle;
        changed = changed || yMaxPos != yMax;
        if (yMaxPos != yMax)
          yMaxPos = yMax;
        //if (changed)
          //sensitivityPos.set(xStartPos, xMiddlePos, yStartPos, yMiddlePos, yMaxPos);

        SmartDashboard.putNumber("Joystick_X", driverController.getLeftY());
        SmartDashboard.putNumber("Joystick_Y", driverController.getLeftX());

        SmartDashboard.putNumber("Joystick_OutX", -sensitivityPos.transfer(driverController.getLeftY()));
        SmartDashboard.putNumber("Joystick_OutY", -sensitivityPos.transfer(driverController.getLeftX()));
        
        //new InstantCommand(() -> drivetrain.resetOdometry(move11.getInitialPose()))
        double maxSpeed = drivetrain.allianceColor == Alliance.Red ? -MaxSpeed : MaxSpeed;
        
        return drive.withVelocityX(
            // Drive forward with negative Y (forward)
            - maxSpeed * sensitivityPos.transfer(driverController.getLeftY())
           //-MaxSpeed * driverController.getLeftY()
          )
          .withVelocityY(
            // Drive left with negative X (left)
            - maxSpeed * sensitivityPos.transfer(driverController.getLeftX())
            //-MaxSpeed * driverController.getLeftX()
          )
          .withRotationalRate(
            MaxAngularRate * sensitivityRot.transfer(-driverController.getRightX())
          );
        }
      )
    );

  //   drivetrain.setDefaultCommand(
  //     // Drivetrain will execute this command periodically
  //     drivetrain.applyRequest(() ->
  //         drive.withVelocityX(0.5) // Drive counterclockwise with negative X (left)
  //     )
  //  );
  //  drivetrain.setDefaultCommand(
  //     // Drivetrain will execute this command periodically
  //     drivetrain.applyRequest(() ->
  //         drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
  //             .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
  //             .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
  //     )
  //  );


    // Idle while the robot is disabled. This ensures the configured
    // neutral mode is applied to the drive motors while disabled.
    final var idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
      drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );

    driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));
//    driverController.b().whileTrue(drivetrain.applyRequest(() ->
//      point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
//    ));

    // // Run SysId routines when holding back/start and X/Y.
    // // Note that each routine should be run exactly once in a single log.
    driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // // reset the field-centric heading on left bumper press
    driverController.rightBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
  
    // Vision control bindings (driver controller only)
    // Left bumper: Drive to AprilTag (vision-guided alignment)
    driverController.leftTrigger(OperatorConstants.TriggerThreshold).whileTrue(new DriveToTag(vision, drivetrain));

    Supplier<Pose2d> goalPoseSupplier = () -> new Pose2d(Units.feetToMeters(5), Units.feetToMeters(3), Rotation2d.fromDegrees(90));
    Supplier<Pose2d> poseProvider = drivetrain::getPose;

  //  driverController.leftTrigger(OperatorConstants.TriggerThreshold).whileTrue(new AlignedDriveToTag(vision, drivetrain, fieldLayout, VisionConstants.Direction.Left,goalPoseSupplier, poseProvider));
  //  driverController.rightTrigger(OperatorConstants.TriggerThreshold).whileTrue(new AlignedDriveToTag(vision, drivetrain, fieldLayout, VisionConstants.Direction.Right, goalPoseSupplier, poseProvider));
//    driverController.y().whileTrue(new AlignedDriveToTag(vision, drivetrain, fieldLayout, VisionConstants.Direction.Center, goalPoseSupplier, poseProvider));

    // if (useTwoControllers)
    //   driverController.leftBumper().whileTrue(new DriveToTag(vision, drivetrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(exampleSubsystem);
  }


  private static SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> createModuleConstants(SwerveModuleConfig constants)
  {
    return TunerConstants.ConstantCreator.createModuleConstants(
      constants.steerMotorId, constants.driveMotorId, constants.encoderId,
      constants.encoderOffset,
      constants.xPos, constants.yPos,
      constants.invertSide, constants.steerMotorInverted, constants.encoderInverted
    );
  }

  // Creates a CommandSwerveDrivetrain instance.
  // This should only be called once in your robot program,.
  private CommandSwerveDrivetrain createDrivetrain(RobotConfig config)
  {
    return new CommandSwerveDrivetrain(
        config, DrivetrainConstants,
        createModuleConstants(config.frontLeft),
        createModuleConstants(config.frontRight),
        createModuleConstants(config.backLeft),
        createModuleConstants(config.backRight)
    );
  }

  /**
   * Gets the vision subsystem (Coral robot only - single camera).
   * @return The VisionSubsystem instance
   */
  public VisionSubsystem getVision() {
    return vision;
  }

    public void resetPose() {
        // Example Only - startPose should be derived from some assumption
        // of where your robot was placed on the field.
        // The first pose in an autonomous path is often a good choice.
        var startPose = new Pose2d(1, 1, new Rotation2d());
        // drivetrain.resetPose(startPose, true);
        // vision.resetSimPose(startPose);
    }
}
