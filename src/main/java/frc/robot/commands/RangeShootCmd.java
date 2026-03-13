// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PositionMech;
import frc.robot.subsystems.VelocityMech;
import frc.robot.subsystems.VelocitySubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/** A command to take Algae into the robot. */
public class RangeShootCmd extends Command {
  class ShooterPreset {
    public double distance;
    public double speed;
    public double elevation;

    public ShooterPreset(double distance, double speed, double elevation)
    {
      this.distance = distance;
      this.speed = speed;
      this.elevation = elevation;
    }
  };

  private final VelocitySubsystem shooter;
  private final PositionMech hood;
  private final VelocityMech feeder;
  private final Supplier<Pose2d> poseProvider;
  private ShooterPreset[] shooterPresets =
  {
    new ShooterPreset(17.5, 5000, 2), //0 -1
    new ShooterPreset(47, 5000, 2), //1.5
    new ShooterPreset(77, 5000, 2), //2.75
    new ShooterPreset(107, 5400, 2), //
    new ShooterPreset(127, 6350, 2),
    new ShooterPreset(147, 6600, 2),
    new ShooterPreset(174, 7000, 2) //
  };
  private Translation2d hubPos = new Translation2d(182.11, 158.84);
  private Pose2d currentPose;

  /**
   * Rolls Algae into the intake.
   *
   * @param roller The subsystem used by this command.
   */
  public RangeShootCmd(VelocitySubsystem shooter, VelocityMech feeder, PositionMech hood, Supplier<Pose2d> poseProvider) {
    this.shooter = shooter;
    this.hood = hood;
    this.feeder = feeder;
    this.poseProvider = poseProvider;
    currentPose = poseProvider.get();
    //shooter.setTargetSpeed(speed);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
//    shooter.setSpeed(ShooterConstants.Speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      Pose2d pose = poseProvider.get();
      if (currentPose != pose)
      {
        Translation2d pos = pose.getTranslation();
        double distance = pos.getDistance(hubPos);

        ShooterPreset preset = null;
        for (int i = 0; i < shooterPresets.length; ++i)
        {
          if (distance <= shooterPresets[i].distance)
          {
            preset = shooterPresets[i];
            break;
          }
        }
        if (preset == null)
        {
            preset = shooterPresets[shooterPresets.length - 1];
        }

        double speed = preset.speed;
        double hoodPos = preset.elevation;

        shooter.setSpeed(speed);
        hood.setPosition(hoodPos);

        feeder.setSpeed(IndexerConstants.FeederSpeed);

        currentPose = pose;
      }
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
