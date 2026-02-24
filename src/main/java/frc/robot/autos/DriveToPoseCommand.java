package frc.robot.autos;

import static edu.wpi.first.units.Units.*;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class DriveToPoseCommand extends Command {
    /** Creates a new DriveToPoseCommand. */
    private static final double TRANSLATION_TOLERANCE = 0.02;
    private static final double THETA_TOLERANCE = Units.degreesToRadians(2.0);
  
    private static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
      Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,
      Constants.AutoConstants.kMaxSpeedMetersPerSecond
    );
    private static final TrapezoidProfile.Constraints DEFAULT_OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
      Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared,
      Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond
    );
  
    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController thetaController;
  
    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<Pose2d> poseProvider;
    private final Supplier<Pose2d> goalPoseSupplier;
    private final boolean useAllianceColor;
  
      private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
      private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors


    public DriveToPoseCommand(CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> goalPoseSupplier, Supplier<Pose2d> poseProvider, boolean useAllianceColor) {
      // Use addRequirements() here to declare subsystem dependencies.
      this.drivetrain = drivetrain;
      this.goalPoseSupplier = goalPoseSupplier;
      this.poseProvider = poseProvider;
      this.useAllianceColor = useAllianceColor;
  
      xController = new ProfiledPIDController(Constants.AutoConstants.kPXController, 0, 0, DEFAULT_XY_CONSTRAINTS);
      yController = new ProfiledPIDController(Constants.AutoConstants.kPYController, 0, 0, DEFAULT_XY_CONSTRAINTS);
      thetaController = new ProfiledPIDController(Constants.AutoConstants.kPThetaController, 0, 0, DEFAULT_OMEGA_CONSTRAINTS);
  
      xController.setTolerance(TRANSLATION_TOLERANCE);
      yController.setTolerance(TRANSLATION_TOLERANCE);
      thetaController.setTolerance(THETA_TOLERANCE);
  
      thetaController.enableContinuousInput(-Math.PI, Math.PI);
  
      addRequirements(drivetrain);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      resetPIDControllers();
      Pose2d pose = goalPoseSupplier.get();

      if (useAllianceColor && drivetrain.allianceColor == DriverStation.Alliance.Red) {
        Translation2d transformedTranslation = new Translation2d(pose.getX(), 8.0137 - pose.getY());
        Rotation2d transformedHeading = pose.getRotation().times(-1);
        pose = new Pose2d(transformedTranslation, transformedHeading);
      }

      thetaController.setGoal(pose.getRotation().getRadians());
      xController.setGoal(pose.getX());
      yController.setGoal(pose.getY());
    }
  
    public boolean atGoal() {
      return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    }
  
    private void resetPIDControllers() {
      Pose2d robotPose = poseProvider.get();
      thetaController.reset(robotPose.getRotation().getRadians());
      xController.reset(robotPose.getX());
      yController.reset(robotPose.getY());
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      Pose2d robotPose = poseProvider.get();
  
      double xSpeed = xController.calculate(robotPose.getX());
      if (xController.atGoal()) {
        xSpeed = 0;
      }
  
      double ySpeed = yController.calculate(robotPose.getY());
      if (yController.atGoal()) {
        ySpeed = 0;
      }
  
      double omegaSpeed = thetaController.calculate(robotPose.getRotation().getRadians());
      if (thetaController.atGoal()) {
        omegaSpeed = 0;
      }
  
      System.out.println("=== Pose: " + drivetrain.getPose().toString() + " Position: " + drivetrain.getPosition().toString());

      drivetrain.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
    //   drivetrain.drive(
    //     ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation())
    //   );
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(drive.withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0));        
//      drivetrain.drive(new Translation2d(), 0.0, false, true);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return atGoal();
    }
  }
