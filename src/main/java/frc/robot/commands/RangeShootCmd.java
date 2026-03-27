package frc.robot.commands;

import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RangeTable;
import frc.robot.Robot;
import frc.robot.subsystems.PositionMech;
import frc.robot.subsystems.VelocityMech;
import frc.robot.subsystems.VelocityMech2;

import java.lang.constant.Constable;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/** A command to take Algae into the robot. */
public class RangeShootCmd extends Command {
  private final VelocityMech2 shooter;
  private final PositionMech hood;
  private final VelocityMech feeder;
  private final VelocityMech indexer;
  private final RangeTable rangeTable;
  private final Supplier<Pose2d> poseProvider;
  private Translation2d blueHubPos = new Translation2d(Units.inchesToMeters(182.11), Units.inchesToMeters(158.84));
  private Translation2d redHubPos = new Translation2d(Units.inchesToMeters(651.22 - 182.11), Units.inchesToMeters(158.84));
  private Pose2d currentPose;
  private boolean running = false;
  private boolean poseChanged = false;
  private double positionTolerance = ShooterConstants.RangePositionTolerance;

  /**
   * Rolls Algae into the intake.
   *
   * @param roller The subsystem used by this command.
   */
  public RangeShootCmd(VelocityMech2 shooter, PositionMech hood, VelocityMech feeder, VelocityMech indexer, RangeTable rangeTable, Supplier<Pose2d> poseProvider) {
    this.shooter = shooter;
    this.hood = hood;
    this.indexer = indexer;
    this.feeder = feeder;
    this.rangeTable = rangeTable;
    this.poseProvider = poseProvider;
    currentPose = poseProvider.get();
    //shooter.setTargetSpeed(speed);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, hood, feeder, indexer);

      Logger.recordOutput("RangeShoot/Pose", currentPose);
      Logger.recordOutput("RangeShoot/Pose", currentPose);
      Logger.recordOutput("RangeShoot/Distance", 0);
      Logger.recordOutput("RangeShoot/RPM", 0);
      Logger.recordOutput("RangeShoot/Hood", 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      running = false;
      poseChanged = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      Pose2d pose = poseProvider.get();
      Translation2d pos = pose.getTranslation();
      double delta = pos.getDistance(currentPose.getTranslation());
      //if (delta >= positionTolerance)
      {
        Translation2d hubPos = Robot.isBlueSide() ? blueHubPos : redHubPos;
        double distance = pos.getDistance(hubPos);

        RangeTable.Range range = rangeTable.getRange(distance);
        double speed = range.speed;
        double hoodPos = range.elevation;
        Logger.recordOutput("RangeShoot/Pose", pose);
        Logger.recordOutput("RangeShoot/Pose", new Pose2d(hubPos, Rotation2d.kZero));
        Logger.recordOutput("RangeShoot/Distance", distance);
        Logger.recordOutput("RangeShoot/RPM", speed);
        Logger.recordOutput("RangeShoot/Hood", hoodPos);

        // shooter.setSpeed(speed);
        // hood.run(hoodPos);

        currentPose = pose;
        poseChanged = true;
      }

      if (shooter.atTarget())// && poseChanged)
      {
        running = true;
        poseChanged = false;
        // indexer.setTargetSpeed(-IndexerConstants.Speed);
        // indexer.setSpeed(-IndexerConstants.Speed);
        // feeder.setTargetSpeed(-IndexerConstants.FeederSpeed);
        // feeder.setSpeed(-IndexerConstants.FeederSpeed);
      }
  }

    // Called once the command ends or is interrupted. This ensures the roller is not running when not intented.
    @Override
    public void end(boolean interrupted)
    {
        indexer.stop();
        feeder.stop();
        running = false;
        poseChanged = false;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return false;
    }
}
