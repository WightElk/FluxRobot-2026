// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VelocityMech;
import edu.wpi.first.wpilibj2.command.Command;

/** A command to take Algae into the robot. */
public class IntakeCommand extends Command {
  private final VelocityMech intake;
  /**
   * Rolls Algae into the intake.
   *
   * @param intake The subsystem used by this command.
   */
  public IntakeCommand(VelocityMech intake, double speed) {
    this.intake = intake;
    intake.setTargetSpeed(speed);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.reset();
    intake.setSpeed(IntakeConstants.InSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //intake.setSpeed(IntakeConstants.InSpeed);
//    intake.run(Constants.Backward);
  }

  // Called once the command ends or is interrupted. This ensures the roller is not running when not intented.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
