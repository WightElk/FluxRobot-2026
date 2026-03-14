package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.subsystems.VelocityMech;
import edu.wpi.first.wpilibj2.command.Command;

/** A command to take Algae into the robot. */
public class IndexerCommand extends Command {
  private final VelocityMech indexer;
  private final int direction;
  /**
   * Rolls Algae into the intake.
   *
   * @param roller The subsystem used by this command.
   */
  public IndexerCommand(VelocityMech indexer, double speed, int dir) {
    this.indexer = indexer;
    direction = dir;
    indexer.setTargetSpeed(speed);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = IndexerConstants.Speed;// * Constants.MaxMotorRPS;
    if (direction != Constants.Forward)
      speed = -speed;
    indexer.setSpeed(speed);
  }

  // Called once the command ends or is interrupted. This ensures the roller is not running when not intented.
  @Override
  public void end(boolean interrupted) {
    indexer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
