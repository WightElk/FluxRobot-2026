package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PositionMech;
import edu.wpi.first.wpilibj2.command.Command;

/** A command to take Algae into the robot. */
public class TiltIntakeCmd extends Command {
  private final PositionMech intake;
  private final int direction;
  /**
   * Rolls Algae into the intake.
   *
   * @param intake The subsystem used by this command.
   */
  public TiltIntakeCmd(PositionMech intake, int direction) {
    this.intake = intake;
    this.direction = direction;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.reset();
    intake.run(direction == Constants.Forward ? IntakeConstants.OutTiltPosition : IntakeConstants.InTiltPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted. This ensures the roller is not running when not intented.
  @Override
  public void end(boolean interrupted) {
    if (interrupted)
      intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
