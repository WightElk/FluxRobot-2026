package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.autos.DriveForwardAuto;
import frc.robot.commands.AlgieInCommand;
import frc.robot.commands.AlgieOutCommand;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.RollerSubsystem;

/**
 * Robot with Algae Intake
 */
public class AlgaeRobotContainer extends RobotContainer {
  private final Pneumatics pneumatics = new Pneumatics();
  public final RollerSubsystem roller = new RollerSubsystem();

  public final DriveForwardAuto autoDriveForward = new DriveForwardAuto(drivetrain);

  private final CommandXboxController operatorController =
    new CommandXboxController(OperatorConstants.OperatorControllerPort);

  AlgaeRobotContainer() {
    super(RobotConfig.AlgaeRobot, false);
    
    configureBindings();
  }

  @Override
  protected void configureBindings() {
    super.configureBindings();

    useTwoControllers = SmartDashboard.getBoolean("Use 2 controllers", OperatorConstants.UseTwoControllers);
    useTwoControllers = false;

    CommandXboxController controller = useTwoControllers ? operatorController : driverController;

    // Pneumatics bindings
    if (useTwoControllers) {
      controller.a().onTrue(new InstantCommand(() -> pneumatics.setForward()));
      controller.b().onTrue(new InstantCommand(() -> pneumatics.setReverse()));
      controller.y().onTrue(new InstantCommand(() -> pneumatics.setOff()));
  //    controller.start().onTrue(new InstantCommand(() -> pneumatics.toggleCompressor()));
      controller.start().onTrue(new InstantCommand(() -> pneumatics.enableCompressor()));
      controller.back().onTrue(new InstantCommand(() -> pneumatics.disableCompressor()));

      controller.leftBumper().whileTrue(new AlgieInCommand(roller));
        
      // Here we use a trigger as a button when it is pushed past a certain threshold
      controller.leftTrigger(OperatorConstants.TriggerThreshold).whileTrue(new AlgieOutCommand(roller));
    }
    else {
      controller.a().onTrue(new InstantCommand(() -> pneumatics.setForward()));
      controller.b().onTrue(new InstantCommand(() -> pneumatics.setReverse()));
      controller.y().onTrue(new InstantCommand(() -> pneumatics.setOff()));
      controller.start().onTrue(new InstantCommand(() -> pneumatics.enableCompressor()));
      controller.back().onTrue(new InstantCommand(() -> pneumatics.disableCompressor()));
  
      controller.rightBumper().whileTrue(new AlgieOutCommand(roller));
      
      // Here we use a trigger as a button when it is pushed past a certain threshold
      controller.rightTrigger(OperatorConstants.TriggerThreshold).whileTrue(new AlgieInCommand(roller));
    }
  }

  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return autoDriveForward;
  }
}
