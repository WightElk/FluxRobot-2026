package frc.robot;

import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.VelocityMech;
import frc.robot.subsystems.VelocitySubsystem;
import frc.robot.subsystems.PositionMech;
import frc.robot.autos.DriveForwardAuto;
import frc.robot.commands.Autos;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.StopShootCommand;

/**
 * Robot with Fuel Shooter
 */
public class FuelRobotContainer extends RobotContainer {
  private final CANBus canBus = new CANBus(RobotConfig.FuelRobot.systemCANBus);
  private final VelocityMech intake;
  private final PositionMech tilter;
  private final VelocityMech indexer;
  private final VelocityMech feeder;
//  private final ShooterSubsystem shooter;
  private final VelocitySubsystem shooter;
  private final PositionMech hood;
//  private final Lights lights = new Lights(RobotConfig.FuelRobot.systemCANBus);

  private final CommandXboxController operatorController =
    new CommandXboxController(OperatorConstants.OperatorControllerPort);

    public final DriveForwardAuto autoDriveForward = new DriveForwardAuto(drivetrain);
    //  public final DrivePathAuto autoDriveCommand = new DrivePathAuto(drivetrain);
    //public final DriveToPoseCommand autoDriveCommand;

    private SendableChooser<Command> autoCommandChooser;

  public FuelRobotContainer() {
    super(RobotConfig.FuelRobot, false);

    int connectedJoystickCount = connectedJoystickCount();
    System.out.println("connectedJoystickCount " + connectedJoystickCount);
    useTwoControllers = connectedJoystickCount == 2;

    // intake = new IntakeSubsystem(IntakeConstants.MotorId, canBus);
    // indexer = new IndexerSubsystem(canBus);
//  shooter = new ShooterSubsystem(canBus)
    intake = new VelocityMech(canBus, "Intake", IntakeConstants.MotorId);
    tilter = new PositionMech(canBus, "Tilter", IntakeConstants.TiltMotorId);
    indexer = new VelocityMech(canBus, "Indexer", IndexerConstants.IndexerId);
    feeder = new VelocityMech(canBus, "Feeder", IndexerConstants.FeederId);
    shooter = new VelocitySubsystem(canBus, "Shooter", ShooterConstants.RightMotorId, ShooterConstants.LeftMotorId);//ShooterConstants.LeftMotorId
    hood = new PositionMech(canBus, "Hood", ShooterConstants.HoodMotorId);

    Supplier<Pose2d> goalPoseSupplier = () -> new Pose2d(Units.feetToMeters(5), Units.feetToMeters(3), Rotation2d.fromDegrees(90));
    Supplier<Pose2d> poseProvider = drivetrain::getPose;

    //autoDriveCommand = new DriveToPoseCommand(drivetrain, goalPoseSupplier, poseProvider, true);
  
    configureBindings();

    storeParameters();
  
    initAutoCommands();
  }

  public void initRobot() {
    //TODO
//    FollowPathCommand.warmupCommand().schedule();
  }

  @Override
  protected void configureBindings() {
    super.configureBindings();

//    SmartDashboard.putBoolean("Use 2 controllers", OperatorConstants.UseTwoControllers);
    // useTwoControllers = SmartDashboard.getBoolean("2 controllers", OperatorConstants.UseTwoControllers);
    // useTwoControllers = false;

    CommandXboxController controller = useTwoControllers ? operatorController : driverController;

    // Intake control
    // Right trigger - Intake IN with variable speed
    // Right bumper - Intake OUT
    //TODO Back
    controller.rightTrigger(OperatorConstants.TriggerThreshold).whileTrue(new IntakeCommand(intake, IntakeConstants.InSpeed));

//    controller.rightTrigger(OperatorConstants.TriggerThreshold).and(controller.rightBumper().negate()).whileTrue(new RunCommand(() -> intake.run(), intake));

    //    controller.rightBumper().whileTrue(new RawIntakeCommand(intake, () -> - IntakeConstants.OutSpeed));
    // controller.povLeft().or(controller.povRight()).whileTrue(new RunCommand(() -> elevator.stop(), elevator));

    // Indexer control
    // Left trigger - Indexer IN with variable speed
    // Left bumper - Indexer OUT
    controller.leftTrigger(OperatorConstants.TriggerThreshold).whileTrue(new FeederCommand(feeder, IndexerConstants.FeederSpeed, Constants.Forward));
    controller.leftBumper().whileTrue(new IndexerCommand(indexer, IndexerConstants.InSpeed, Constants.Backward));

    // controller.leftTrigger(OperatorConstants.TriggerThreshold).whileTrue(new RunCommand(() -> indexer.run(Constants.Forward), indexer));
    // controller.leftBumper().whileTrue(new RunCommand(() -> indexer.run(Constants.Backward), indexer));

    // Shooter control
    // A - Shooter ON
    // B - Shooter OFF
//      controller.rightTrigger(OperatorConstants.TriggerThreshold).whileTrue(new RawShooterCommand(shooter, () -> controller.getLeftTriggerAxis() - OperatorConstants.TriggerThreshold));
//    controller.rightTrigger(OperatorConstants.TriggerThreshold).and(controller.rightBumper()).whileTrue(new RawShooterCommand(shooter, () -> (controller.getRightTriggerAxis() )));

//toggleOnTrue
    controller.a().onTrue(new ShootCommand(shooter, ShooterConstants.Speed));
    controller.b().onTrue(new StopShootCommand(shooter));

    controller.povUp().whileTrue(new RunCommand(() -> hood.jogUp(ShooterConstants.HoodStep), hood));
    controller.povDown().whileTrue(new RunCommand(() -> hood.jogDown(ShooterConstants.HoodStep), hood));

    // Push out
    controller.povLeft().whileTrue(new RunCommand(() -> tilter.jogDown(IntakeConstants.TiltStep), tilter));
    // Pull in
    controller.povRight().whileTrue(new RunCommand(() -> tilter.jogUp(IntakeConstants.TiltStep), tilter));

    //controller.povUp().whileTrue(new RunCommand(() -> hood.setPosition(0), hood));
    // controller.povLeft().or(controller.povRight()).whileTrue(new RunCommand(() -> elevator.stop(), elevator));

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
  }

  public Command getAutonomousCommand() {
    // Build an auto chooser. This will use Commands.none() as the default option.
    //autoCommandChooser = AutoBuilder.buildAutoChooser();
    // Another option that allows you to specify the default auto by its name
    // autoCommandChooser = AutoBuilder.buildAutoChooser("My Default Auto");

//    return autoCommandChooser.getSelected();

//    return Autos.exampleAuto(exampleSubsystem);

    // The selected command will be run in autonomous
    return autoDriveForward;
//    return autoDriveCommand.andThen((new RunCommand(() -> elevator.moveToLevel1(), elevator)).withTimeout(2.0)).andThen(new RawTrayCommand(tray, () -> -TrayConstants.Speed));
  }

  public void storeParameters()
  {
    System.out.println("storeParameters");

    intake.putParams();
    tilter.putParams();
    indexer.putParams();
    feeder.putParams();
    shooter.putParams();
    hood.putParams();

    SmartDashboard.putData(autoCommandChooser);
  }

  public void fetchParameters()
  {
    System.out.println("fetchParameters");

    intake.getParams();
    tilter.getParams();
    indexer.getParams();
    feeder.getParams();
    shooter.getParams();
    hood.getParams();
  }

  protected int connectedJoystickCount()
  {
    int connectedJoystickCount = 0;
    for (int i = 0; i < 6; ++i)
      if (DriverStation.isJoystickConnected(i))
        connectedJoystickCount++;
    return connectedJoystickCount;
  }

  protected void initAutoCommands()
  {
    if (Constants.AutoConstants.commands.length > 0)
    {
      Command cmd = new PathPlannerAuto(Constants.AutoConstants.commands[0][1]);
      autoCommandChooser.setDefaultOption(Constants.AutoConstants.commands[0][0], cmd);
    }

    for (int i = 1; i < Constants.AutoConstants.commands.length; ++i)
    {
      Command cmd = new PathPlannerAuto(Constants.AutoConstants.commands[i][1]);
      autoCommandChooser.addOption(Constants.AutoConstants.commands[i][0], cmd);
    }
  }

  protected void createPath() {
    // Create a list of waypoints from poses. Each pose represents one waypoint.
    // The rotation component of the pose should be the direction of travel. Do not use holonomic rotation.
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
            new Pose2d(3.0, 1.0, Rotation2d.fromDegrees(0)),
            new Pose2d(5.0, 3.0, Rotation2d.fromDegrees(90))
    );

    PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
    // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

    // Create the path using the waypoints created above
    PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
            new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;
  }

}
