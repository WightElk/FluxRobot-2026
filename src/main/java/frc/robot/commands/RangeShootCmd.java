package frc.robot.commands;

import frc.robot.Constants.IndexerConstants;
import frc.robot.RangeTable;
import frc.robot.subsystems.PositionMech;
import frc.robot.subsystems.VelocityMech;
import frc.robot.subsystems.VelocityMech2;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** A command to take Algae into the robot. */
public class RangeShootCmd extends Command {
  private final VelocityMech2 shooter;
  private final PositionMech hood;
  private final VelocityMech feeder;
  private final RangeTable rangeTable;
  private final Supplier<Pose2d> poseProvider;
  private Translation2d hubPos = new Translation2d(182.11, 158.84);
  private Pose2d currentPose;

  /**
   * Rolls Algae into the intake.
   *
   * @param roller The subsystem used by this command.
   */
  public RangeShootCmd(VelocityMech2 shooter, PositionMech hood, VelocityMech feeder, RangeTable rangeTable, Supplier<Pose2d> poseProvider) {
    this.shooter = shooter;
    this.hood = hood;
    this.feeder = feeder;
    this.rangeTable = rangeTable;
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

        RangeTable.Range range = rangeTable.getRange(distance);
        double speed = range.speed;
        double hoodPos = range.elevation;

        shooter.setSpeed(speed);
        hood.run(hoodPos);

        feeder.setSpeed(IndexerConstants.FeederSpeed);

        currentPose = pose;
      }

      if (shooter.atSetPoint() && hood.atTarget())
      {
        feeder.setSpeed(IndexerConstants.FeederSpeed);
      }

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
    public void end(boolean interrupted)
    {
        feeder.stop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
