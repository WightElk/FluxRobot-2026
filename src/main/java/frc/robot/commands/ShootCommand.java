// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.VelocitySubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** A command to take Algae into the robot. */
public class ShootCommand extends Command {
  private final VelocitySubsystem shooter;
  /**
   * Rolls Algae into the intake.
   *
   * @param roller The subsystem used by this command.
   */
  public ShootCommand(VelocitySubsystem shooter, double speed) {
    this.shooter = shooter;
    //shooter.setTargetSpeed(speed);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setSpeed(ShooterConstants.Speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // shooter.setSpeed(ShooterConstants.Speed);

        // if (running && targetVelocityChanged)
        // {
        //     velocityRPM = direction == Constants.Backward ? -targetVelocity : targetVelocity;
        //     motor.setControl(velocityVoltage.withVelocity(velocityRPM / 60.0));

        //     double speed;
        //     shooter.setSpeed(speed);
        //     System.out.println("setControl-periodic");
        // }

        // double vel = 60 * getVelocity();
        // if (vel != velocity)
        // {
        //     String prefix = name + "/";
        //     SmartDashboard.putNumber(prefix + "RPM", vel);
        //     velocity = vel;
        // }
        // double target = direction == Constants.Backward ? -targetVelocity : targetVelocity;
        // atSpeed = Math.abs(vel - target) <= rpmDelta;
        // //System.out.println("RPM: " + velocityRPM + " / " + vel);
  }

  // Called once the command ends or is interrupted. This ensures the roller is not running when not intented.
  @Override
  public void end(boolean interrupted) {
    //shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  //
}
