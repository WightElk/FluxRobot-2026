package frc.robot.commands;

import frc.robot.RangeTable;
import frc.robot.subsystems.PositionMech;
import frc.robot.subsystems.VelocityMech2;

import edu.wpi.first.wpilibj2.command.Command;

/** A command to take Algae into the robot. */
public class SetShooterRangeCmd extends Command {
  private final VelocityMech2 shooter;
  private final PositionMech hood;
  private final RangeTable rangeTable;
  private final int range;

  /**
   * Rolls Algae into the intake.
   *
   * @param roller The subsystem used by this command.
   */
  public SetShooterRangeCmd(VelocityMech2 shooter, PositionMech hood, RangeTable rangeTable, int range) {
    this.shooter = shooter;
    this.hood = hood;
    this.range = range;
    this.rangeTable = rangeTable;

    addRequirements(shooter, hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double speed = rangeTable.getSpeedPreset(range);
    double hoodPos = rangeTable.getElevationPreset(range);;
    shooter.setSpeed(speed);
    hood.run(hoodPos);
  }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute()
    {
    }

    // Called once the command ends or is interrupted. This ensures the roller is not running when not intented.
    @Override
    public void end(boolean interrupted)
    {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
