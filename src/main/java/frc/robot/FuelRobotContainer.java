package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.TrayConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VelocitySubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Lights;
import frc.robot.commands.DriveToTag;
import frc.robot.commands.RawIndexerCommand;
import frc.robot.commands.RawIntakeCommand;
import frc.robot.commands.RawShooterCommand;
import frc.robot.commands.TrayInOutCommand;
import frc.robot.commands.RawTrayCommand;
import frc.robot.autos.DriveForwardAuto;
import frc.robot.autos.DrivePathAuto;
import frc.robot.autos.DriveToPoseCommand;

/**
 * Robot with Fuel Shooter
 */
public class FuelRobotContainer extends RobotContainer {
  private final CANBus canBus = new CANBus(RobotConfig.FuelRobot.systemCANBus);
  private final IntakeSubsystem intake;
  private final IndexerSubsystem indexer;
//  private final ShooterSubsystem shooter;
  private final VelocitySubsystem shooter;
//  private final Lights lights = new Lights(RobotConfig.FuelRobot.systemCANBus);

  private final CommandXboxController operatorController =
    new CommandXboxController(OperatorConstants.OperatorControllerPort);

    public final DriveForwardAuto autoDriveForward = new DriveForwardAuto(drivetrain);
    //  public final DrivePathAuto autoDriveCommand = new DrivePathAuto(drivetrain);
    //public final DriveToPoseCommand autoDriveCommand;

  public FuelRobotContainer() {
    super(RobotConfig.FuelRobot, false);

    intake = new IntakeSubsystem(IntakeConstants.MotorId, canBus);
    indexer = new IndexerSubsystem(canBus);
//  shooter = new ShooterSubsystem(canBus);
    shooter = new VelocitySubsystem(canBus, "Shooter", Constants.ShooterConstants.RightMotorId, -1);

    Supplier<Pose2d> goalPoseSupplier = () -> new Pose2d(Units.feetToMeters(5), Units.feetToMeters(3), Rotation2d.fromDegrees(90));
    Supplier<Pose2d> poseProvider = drivetrain::getPose;

    //autoDriveCommand = new DriveToPoseCommand(drivetrain, goalPoseSupplier, poseProvider, true);
  
    configureBindings();
  }

  @Override
  protected void configureBindings() {
    super.configureBindings();

//    SmartDashboard.putBoolean("Use 2 controllers", OperatorConstants.UseTwoControllers);

    useTwoControllers = SmartDashboard.getBoolean("Use 2 controllers", OperatorConstants.UseTwoControllers);
    useTwoControllers = false;

    CommandXboxController controller = useTwoControllers ? operatorController : driverController;

    // Intake control
    // Right trigger - Intake IN with variable speed
    // Right bumper - Intake OUT
    controller.rightTrigger(OperatorConstants.TriggerThreshold).and(controller.rightBumper().negate()).whileTrue(new RawIntakeCommand(intake, () -> (controller.getRightTriggerAxis() - OperatorConstants.TriggerThreshold)));
//    controller.rightBumper().whileTrue(new RawIntakeCommand(intake, () -> - IntakeConstants.OutSpeed));
    // controller.povLeft().or(controller.povRight()).whileTrue(new RunCommand(() -> elevator.stop(), elevator));

    // Indexer control
    // Left trigger - Indexer IN with variable speed
    // Left bumper - Indexer OUT
    controller.leftTrigger(OperatorConstants.TriggerThreshold).whileTrue(new RawIndexerCommand(indexer, () -> (controller.getLeftTriggerAxis())));
    controller.leftBumper().whileTrue(new RawIndexerCommand(indexer, () -> IndexerConstants.BackwardSpeed));

    // Shooter control
    // A - Shooter ON
    // B - Shooter OFF
//      controller.rightTrigger(OperatorConstants.TriggerThreshold).whileTrue(new RawShooterCommand(shooter, () -> controller.getLeftTriggerAxis() - OperatorConstants.TriggerThreshold));

//    controller.rightTrigger(OperatorConstants.TriggerThreshold).and(controller.rightBumper()).whileTrue(new RawShooterCommand(shooter, () -> (controller.getRightTriggerAxis() )));

//    controller.a().onTrue(new RunCommand(() -> shooter.setSpeed(0.5), shooter));
    controller.back().onTrue(new RunCommand(() -> shooter.getParams(), shooter));

    controller.a().onTrue(new RunCommand(() -> shooter.run(), shooter));
    controller.b().onTrue(new RunCommand(() -> shooter.stop(), shooter));

    // controller.povDown().whileTrue(new RunCommand(() -> elevator.jogUp(), elevator));
    // controller.povUp().whileTrue(new RunCommand(() -> elevator.jogDown(), elevator));
    // controller.povLeft().or(controller.povRight()).whileTrue(new RunCommand(() -> elevator.stop(), elevator));

    // controller.y().onTrue(new RunCommand(() -> elevator.moveToLevel2(), elevator));
    // controller.b().onTrue(new RunCommand(() -> elevator.moveToLevel1(), elevator));
  }

  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return autoDriveForward;
//    return autoDriveCommand.andThen((new RunCommand(() -> elevator.moveToLevel1(), elevator)).withTimeout(2.0)).andThen(new RawTrayCommand(tray, () -> -TrayConstants.Speed));
  }
}
