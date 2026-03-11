// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VelocityMech;
import frc.robot.subsystems.VelocitySubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** A command to take Algae into the robot. */
public class FeederCommand extends Command {
  private final VelocityMech feeder;
  private final int direction;
  /**
   * Rolls Algae into the intake.
   *
   * @param roller The subsystem used by this command.
   */
  public FeederCommand(VelocityMech feeder, double speed, int dir) {
    this.feeder = feeder;
    direction = dir;
    feeder.setTargetSpeed(speed);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = IndexerConstants.FeederSpeed;// * Constants.MaxMotorRPS;
    if (direction != Constants.Forward)
      speed = -speed;
    feeder.setSpeed(speed);
  }

  // Called once the command ends or is interrupted. This ensures the roller is not running when not intented.
  @Override
  public void end(boolean interrupted) {
    feeder.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
