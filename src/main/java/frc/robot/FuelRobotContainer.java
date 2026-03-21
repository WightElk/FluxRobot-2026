package frc.robot;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.VelocityMech2;
import frc.robot.subsystems.VelocitySubsystem;
import frc.robot.subsystems.PositionMech;
import frc.robot.subsystems.VelocityMech;
import frc.robot.autos.DriveForwardAuto;
import frc.robot.commands.Autos;
import frc.robot.commands.FeederCommand;
import frc.robot.commands.IndexerCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RangeShootCmd;
import frc.robot.commands.SetShooterRangeCmd;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShootToHubCmd;
import frc.robot.commands.StopShootCommand;
import frc.robot.commands.TiltIntakeCmd;
import frc.robot.commands.VelocityCmd;

/**
 * Robot with Fuel Shooter
 */
public class FuelRobotContainer extends RobotContainer {
  private final CANBus canBus = new CANBus(RobotConfig.FuelRobot.systemCANBus);
  private final VelocityMech intake;
  private final PositionMech tilter;
  private final VelocityMech indexer;
  private final VelocityMech feeder;
  private final VelocityMech2 shooter;
  private final PositionMech hood;
  private final RangeTable rangeTable;
//  private final Lights lights = new Lights(RobotConfig.FuelRobot.systemCANBus);

  private final CommandXboxController operatorController =
    new CommandXboxController(OperatorConstants.OperatorControllerPort);

    public final DriveForwardAuto autoDriveForward = new DriveForwardAuto(drivetrain);
    
    private SendableChooser<Command> autoCommandChooser;
    //@AutoLogOutput(key = "PathChooser")
    private SendableChooser<String> pathChooser;
    private SendableChooser<Pose2d> initPoseChooser;

    public boolean releaseVersion = true;

  public FuelRobotContainer(boolean visionEnabled, boolean releaseVersion) {
    super(RobotConfig.FuelRobot, visionEnabled);
    this.releaseVersion = releaseVersion;

    int connectedJoystickCount = connectedJoystickCount();
    useTwoControllers = releaseVersion ? true : (connectedJoystickCount == 2);
//    System.out.println("connectedJoystickCount " + connectedJoystickCount);

    intake = new VelocityMech(canBus, "Intake", IntakeConstants.MotorId);
    tilter = new PositionMech(canBus, "Tilter", IntakeConstants.TiltMotorId);
    indexer = new VelocityMech(canBus, "Indexer", IndexerConstants.IndexerId);
    feeder = new VelocityMech(canBus, "Feeder", IndexerConstants.FeederId);
    shooter = new VelocityMech2(canBus, "Shooter", ShooterConstants.RightMotorId, ShooterConstants.LeftMotorId);
    hood = new PositionMech(canBus, "Hood", ShooterConstants.HoodMotorId);

    rangeTable = new RangeTable();

    Supplier<Pose2d> goalPoseSupplier = () -> new Pose2d(Units.feetToMeters(5), Units.feetToMeters(3), Rotation2d.fromDegrees(90));
//    Supplier<Pose2d> poseProvider = drivetrain::getPose;

    //autoDriveCommand = new DriveToPoseCommand(drivetrain, goalPoseSupplier, poseProvider, true);
      // Build an auto chooser. This will use Commands.none() as the default option.
    initPosition();
    initPaths();
    initAutoCommands();

    configureBindings();

    resetEncoders();
    drivetrain.setInitPose(initPoseChooser.getSelected());
    
    storeParameters();
    }

  public void initRobot() {
    FollowPathCommand.warmupCommand().schedule();
  }

  @Override
  protected void configureBindings() {
    super.configureBindings();

    CommandXboxController controller = useTwoControllers ? operatorController : driverController;

    if (useTwoControllers)
    {
      // X - Keep robot in place
      driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));

      driverController.y().onTrue(Commands.runOnce(drivetrain::resetGyro, drivetrain));

      // Intake control
      // Right Trigger - Run intake rolller IN
      // Right Bumper - Run intake rolller OUT
      driverController.rightTrigger(OperatorConstants.TriggerThreshold).whileTrue(new VelocityCmd(intake, () -> IntakeConstants.InSpeed, Constants.Forward));
      driverController.rightBumper().whileTrue(new VelocityCmd(intake, () -> IntakeConstants.OutSpeed, Constants.Backward));

      // Intake Tilt control
      // A - Deploy intake
      // B - Retract intake
      driverController.b().onTrue(new TiltIntakeCmd(tilter, Constants.Forward));
      driverController.a().onTrue(new TiltIntakeCmd(tilter, Constants.Backward));
      // Pov Left - Push out intake
      // Pov Down - Pull in intake
      driverController.povLeft().whileTrue(new RunCommand(() -> tilter.jogDown(IntakeConstants.TiltStep), tilter));
      driverController.povRight().whileTrue(new RunCommand(() -> tilter.jogUp(IntakeConstants.TiltStep), tilter));

      driverController.leftBumper().whileTrue(Commands.runOnce(drivetrain::seedFieldCentric, drivetrain));

      driverController.back().onTrue(Commands.runOnce(() -> resetEncoders(), drivetrain));

      // driverController.back().and(controller.leftBumper()).onTrue(Commands.runOnce(() -> resetEncoders()));
      // // Fetch parameters
      // driverController.back().and(controller.leftBumper().negate()).toggleOnTrue(Commands.runOnce(() -> fetchParameters()));
      // Update parameters
      //controller.back().and(controller.x()).toggleOnTrue(Commands.runOnce(() -> storeParameters()));

      // Shooter and Feeder control
      // Right Trigger - Run Feeder and Shoot
      controller.rightTrigger(OperatorConstants.TriggerThreshold).whileTrue(new VelocityCmd(feeder, () -> IndexerConstants.FeederSpeed, Constants.Backward));
      // Left Trigger  - Aim at Hub then Run Feeder and Shoot
      controller.rightBumper().whileTrue(new VelocityCmd(indexer, () -> IndexerConstants.Speed, Constants.Forward));

      controller.leftBumper().whileTrue(new RangeShootCmd(shooter, hood, feeder, indexer, rangeTable, drivetrain::getPose));
      controller.leftTrigger(OperatorConstants.TriggerThreshold).whileTrue(new ShootToHubCmd(shooter, hood, feeder, indexer, drivetrain, drivetrain::getPose));

      // controller.rightTrigger(OperatorConstants.TriggerThreshold).whileTrue(Commands.parallel(
      //   new VelocityCmd(feeder, () -> IndexerConstants.FeederSpeed, Constants.Backward),
      //   new VelocityCmd(indexer, () -> IndexerConstants.Speed, Constants.Forward)));

      // Shooter control
      // A - Shooter ON
      // B - Shooter OFF
      controller.a().onTrue(new SetShooterRangeCmd(shooter, hood, rangeTable, ShooterConstants.ShortRange));
      controller.b().onTrue(new SetShooterRangeCmd(shooter, hood, rangeTable, ShooterConstants.MidRange));
      controller.y().onTrue(new SetShooterRangeCmd(shooter, hood, rangeTable, ShooterConstants.LongRange));

      // Shooter control
      // Start - Toggles Shooter ON/OFF
      controller.start().toggleOnTrue(new ShootCommand(shooter, () -> ShooterConstants.Speed, Constants.Forward));

      // Indexer control
      // POV Right - Indexer rollers IN
      // POV Left  - Indexer rollers OUT
      //controller.povRight().and(controller.leftBumper()).whileTrue(Commands.runOnce(() -> shooter.speedUp(ShooterConstants.SpeedStep), shooter));
      //controller.povLeft().and(controller.leftBumper()).whileTrue(Commands.runOnce(() -> shooter.speedDown(ShooterConstants.SpeedStep), shooter));

      // Shooter Hood
      // Pov Up - Hood Up
      // Pov Down - Hood Down
      controller.povUp().whileTrue(new RunCommand(() -> hood.jogUp(ShooterConstants.HoodStep), hood));
      controller.povDown().whileTrue(new RunCommand(() -> hood.jogDown(ShooterConstants.HoodStep), hood));
    }
    else
    {
      //controller.y().onTrue(Commands.runOnce(drivetrain::resetGyro));
      //controller.x().and(controller.leftBumper()).onTrue(Commands.runOnce(() -> resetEncoders()));
      controller.x().onTrue(Commands.runOnce(() -> resetEncoders()));

      // Intake control
      // Right Trigger - Run intake rolller
      controller.rightTrigger(OperatorConstants.TriggerThreshold).whileTrue(new VelocityCmd(intake, () -> IntakeConstants.InSpeed, Constants.Forward));

      // Intake Tilt control
      // Pov Left - Push out intake
      // Pov Down - Pull in intake
      controller.povLeft().and(controller.leftBumper().negate()).whileTrue(new RunCommand(() -> tilter.jogDown(IntakeConstants.TiltStep), tilter));
      controller.povRight().and(controller.leftBumper().negate()).whileTrue(new RunCommand(() -> tilter.jogUp(IntakeConstants.TiltStep), tilter));

      // controller.povRight().and(controller.leftBumper()).whileTrue(Commands.runOnce(() -> shooter.speedUp(ShooterConstants.SpeedStep), shooter));
      // controller.povLeft().and(controller.leftBumper()).whileTrue(Commands.runOnce(() -> shooter.speedDown(ShooterConstants.SpeedStep), shooter));

      if (false)
      {
        controller.povRight().whileTrue(Commands.runOnce(() -> shooter.speedUp(ShooterConstants.SpeedStep), shooter));
        controller.povLeft().whileTrue(Commands.runOnce(() -> shooter.speedDown(ShooterConstants.SpeedStep), shooter));
      }
      // Feeder control
      // Left Trigger - Run Feeder and Shoot
      controller.leftTrigger(OperatorConstants.TriggerThreshold).whileTrue(new VelocityCmd(feeder, () -> IndexerConstants.FeederSpeed, Constants.Backward));

      // Indexer control
      // Left Bumper - Run Indexer
      controller.leftBumper().and(controller.x().negate()).whileTrue(new VelocityCmd(indexer, () -> IndexerConstants.Speed, Constants.Forward));

      // Shooter control
      // A - Shooter ON
      // B - Shooter OFF
      // controller.a().onTrue(new ShootCommand(shooter, () -> ShooterConstants.Speed, Constants.Forward));
      // controller.b().onTrue(Commands.runOnce(shooter::stop));
      controller.start().toggleOnTrue(new ShootCommand(shooter, () -> ShooterConstants.Speed, Constants.Forward));
//      controller.start().toggleOnFalse(Commands.runOnce(shooter::stop));

      // Shooter control
      // A - Short Range Shooter
      // B - Mid Shooter
      // Y - Long Shooter
      controller.a().onTrue(new SetShooterRangeCmd(shooter, hood, rangeTable, ShooterConstants.ShortRange));
      controller.b().onTrue(new SetShooterRangeCmd(shooter, hood, rangeTable, ShooterConstants.MidRange));
      controller.y().onTrue(new SetShooterRangeCmd(shooter, hood, rangeTable, ShooterConstants.LongRange));

      // Shooter Hood
      // Pov Up - Hood Up
      // Pov Down - Hood Down
      controller.povUp().whileTrue(new RunCommand(() -> hood.jogUp(ShooterConstants.HoodStep), hood));
      controller.povDown().whileTrue(new RunCommand(() -> hood.jogDown(ShooterConstants.HoodStep), hood));

//      controller.rightBumper().whileTrue(new RangeShootCmd(shooter, hood, feeder, rangeTable, drivetrain::getPose));

      controller.rightBumper().whileTrue(drivetrain.followPathCommand(pathChooser.getSelected()));

      // Fetch parameters
      controller.back().and(controller.x().negate()).toggleOnTrue(Commands.runOnce(() -> fetchParameters()));
      // Update parameters
      controller.back().and(controller.leftBumper()).toggleOnTrue(Commands.runOnce(() -> storeParameters()));
    }
  }

  public Command getAutonomousCommand() {
    // Another option that allows you to specify the default auto by its name
    // autoCommandChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    return autoCommandChooser.getSelected();
    //return null;

//    return Autos.exampleAuto(exampleSubsystem);
    // The selected command will be run in autonomous
//    return autoDriveForward;
//    return autoDriveCommand.andThen((new RunCommand(() -> elevator.moveToLevel1(), elevator)).withTimeout(2.0)).andThen(new RawTrayCommand(tray, () -> -TrayConstants.Speed));
  }

  public void resetEncoders()
  {
    drivetrain.setInitPose(Constants.Paths.InitPos.getX(), yMaxPos, fieldLength);
    hood.resetEncoders();
    tilter.resetEncoders();
  }
  
  public void storeParameters()
  {
//    System.out.println("storeParameters");

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
//    System.out.println("fetchParameters");

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

  protected void initPosition()
  {
    initPoseChooser = new SendableChooser<>();

    for (int i = 0; i < Constants.Paths.InitNames.length; ++i)
    {
      Pose2d pose = new Pose2d(Constants.Paths.InitPositions[i], Constants.Paths.InitRotations[i]);
      if (i == 0)
        initPoseChooser.setDefaultOption(Constants.Paths.InitNames[i], pose);
      else
        initPoseChooser.addOption(Constants.Paths.InitNames[i], pose);
    }
    SmartDashboard.putData("Initial Position", initPoseChooser);
  }

  protected void initPaths()
  {
    pathChooser = new SendableChooser<>();

    for (int i = 1; i < Constants.Paths.Paths.length; ++i)
    {
      pathChooser.addOption(Constants.Paths.Paths[i], Constants.Paths.Paths[i]);
    }
  }

  protected void initAutoCommands()
  {
    autoCommandChooser = AutoBuilder.buildAutoChooser();

    autoCommandChooser.setDefaultOption("No Auto", Commands.runOnce(() -> {}));

    for (int i = 0; i < Constants.AutoConstants.commands.length; ++i)
    {
      Command cmd = new PathPlannerAuto(Constants.AutoConstants.commands[i][1]);
      autoCommandChooser.addOption(Constants.AutoConstants.commands[i][0], cmd);
    }
    SmartDashboard.putData("Auto Mode", autoCommandChooser);
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
