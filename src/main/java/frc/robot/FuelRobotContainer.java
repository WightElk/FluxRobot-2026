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
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.VelocitySubsystem;
import frc.robot.subsystems.PositionSubsystem;
import frc.robot.autos.DriveForwardAuto;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.StopShootCommand;

/**
 * Robot with Fuel Shooter
 */
public class FuelRobotContainer extends RobotContainer {
  private final CANBus canBus = new CANBus(RobotConfig.FuelRobot.systemCANBus);
  private final VelocitySubsystem intake;
  private final VelocitySubsystem indexer;
//  private final ShooterSubsystem shooter;
  private final VelocitySubsystem shooter;
  private final PositionSubsystem hood;
//  private final Lights lights = new Lights(RobotConfig.FuelRobot.systemCANBus);

  private final CommandXboxController operatorController =
    new CommandXboxController(OperatorConstants.OperatorControllerPort);

    public final DriveForwardAuto autoDriveForward = new DriveForwardAuto(drivetrain);
    //  public final DrivePathAuto autoDriveCommand = new DrivePathAuto(drivetrain);
    //public final DriveToPoseCommand autoDriveCommand;

  public FuelRobotContainer() {
    super(RobotConfig.FuelRobot, false);

    // intake = new IntakeSubsystem(IntakeConstants.MotorId, canBus);
    // indexer = new IndexerSubsystem(canBus);
//  shooter = new ShooterSubsystem(canBus);
    intake = new VelocitySubsystem(canBus, "Intake", IntakeConstants.MotorId, -1);
    indexer = new VelocitySubsystem(canBus, "Indexer", IndexerConstants.MotorId, IndexerConstants.FollowerId);
    shooter = new VelocitySubsystem(canBus, "Shooter", ShooterConstants.RightMotorId, ShooterConstants.LeftMotorId);//ShooterConstants.LeftMotorId
    hood = new PositionSubsystem(canBus, "Hood", ShooterConstants.HoodMotorId, -1);

    Supplier<Pose2d> goalPoseSupplier = () -> new Pose2d(Units.feetToMeters(5), Units.feetToMeters(3), Rotation2d.fromDegrees(90));
    Supplier<Pose2d> poseProvider = drivetrain::getPose;

    //autoDriveCommand = new DriveToPoseCommand(drivetrain, goalPoseSupplier, poseProvider, true);
  
    configureBindings();

    storeParameters();
  }

  @Override
  protected void configureBindings() {
    super.configureBindings();

//    SmartDashboard.putBoolean("Use 2 controllers", OperatorConstants.UseTwoControllers);

    useTwoControllers = SmartDashboard.getBoolean("2 controllers", OperatorConstants.UseTwoControllers);
    useTwoControllers = false;

    CommandXboxController controller = useTwoControllers ? operatorController : driverController;

    // Intake control
    // Right trigger - Intake IN with variable speed
    // Right bumper - Intake OUT
    controller.rightTrigger(OperatorConstants.TriggerThreshold).whileTrue(new IntakeCommand(intake));

//    controller.rightTrigger(OperatorConstants.TriggerThreshold).and(controller.rightBumper().negate()).whileTrue(new RunCommand(() -> intake.run(), intake));

    //    controller.rightBumper().whileTrue(new RawIntakeCommand(intake, () -> - IntakeConstants.OutSpeed));
    // controller.povLeft().or(controller.povRight()).whileTrue(new RunCommand(() -> elevator.stop(), elevator));

    // Indexer control
    // Left trigger - Indexer IN with variable speed
    // Left bumper - Indexer OUT
    controller.leftTrigger(OperatorConstants.TriggerThreshold).whileTrue(new IndexerCommand(indexer, Constants.Backward));
    controller.leftBumper().whileTrue(new IndexerCommand(indexer, Constants.Backward));

    // controller.leftTrigger(OperatorConstants.TriggerThreshold).whileTrue(new RunCommand(() -> indexer.run(Constants.Forward), indexer));
    // controller.leftBumper().whileTrue(new RunCommand(() -> indexer.run(Constants.Backward), indexer));

    // Shooter control
    // A - Shooter ON
    // B - Shooter OFF
//      controller.rightTrigger(OperatorConstants.TriggerThreshold).whileTrue(new RawShooterCommand(shooter, () -> controller.getLeftTriggerAxis() - OperatorConstants.TriggerThreshold));
//    controller.rightTrigger(OperatorConstants.TriggerThreshold).and(controller.rightBumper()).whileTrue(new RawShooterCommand(shooter, () -> (controller.getRightTriggerAxis() )));

// InstantCommand
// StartEndCommand
    controller.start().toggleOnTrue(new Command() {
        @Override public void initialize() {
          fetchParameters();    
        }
        @Override public boolean isFinished() {
          return true;
        }
    });
    controller.back().toggleOnTrue(new Command() {
        @Override public void initialize() {
          storeParameters();
        }
        @Override public void execute() {
           System.out.println("execute");
        }
        @Override public boolean isFinished() {
          return true;
        }
    });

//toggleOnTrue
    controller.a().onTrue(new ShootCommand(shooter));
    controller.b().onTrue(new StopShootCommand(shooter));

    controller.povDown().whileTrue(new RunCommand(() -> hood.jogDown(), hood));
    controller.povUp().whileTrue(new RunCommand(() -> hood.jogUp(), hood));
    // controller.povLeft().or(controller.povRight()).whileTrue(new RunCommand(() -> elevator.stop(), elevator));
  }

  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return autoDriveForward;
//    return autoDriveCommand.andThen((new RunCommand(() -> elevator.moveToLevel1(), elevator)).withTimeout(2.0)).andThen(new RawTrayCommand(tray, () -> -TrayConstants.Speed));
  }

  public void storeParameters()
  {
    System.out.println("storeParameters");

    intake.putParams();
    indexer.putParams();
    shooter.putParams();
    hood.putParams();
  }

  public void fetchParameters()
  {
    System.out.println("fetchParameters");

    intake.getParams();
    // indexer.getParams();
    // shooter.getParams();
    // hood.getParams();
  }
}
