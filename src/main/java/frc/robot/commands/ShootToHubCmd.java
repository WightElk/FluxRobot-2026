package frc.robot.commands;

import java.util.function.Supplier;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Measure.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RangeTable;
import frc.robot.Robot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PositionMech;
import frc.robot.subsystems.VelocityMech;
import frc.robot.subsystems.VelocityMech2;

public class ShootToHubCmd extends Command
{
    private final VelocityMech2 shooter;
    private final PositionMech hood;
    private final VelocityMech feeder;
    private final VelocityMech indexer;
    private final CommandSwerveDrivetrain drivetrain;
    private final Supplier<Pose2d> poseProvider;
    private final RangeTable rangeTable;

    private Pose2d currentPose;
    private boolean running = false;
    private boolean poseChanged = false;
    private double positionTolerance = ShooterConstants.RangePositionTolerance;
    private Translation2d blueHubPos = new Translation2d(Units.inchesToMeters(82.11), Units.inchesToMeters(158.84));
    private Translation2d redHubPos = new Translation2d(Units.inchesToMeters(651.22 - 182.11), Units.inchesToMeters(158.84));

    private double MaxSpeed = Constants.MaxSpeedCoef * TunerConstants.kSpeedAt12Volts.in(Meters.per(Second)); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(Radians.per(Second)); // 3/4 of a rotation per second max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage) // Use open-loop control for drive motors
        .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);  //OperatorPerspective

    protected final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    public ShootToHubCmd(VelocityMech2 shooter, PositionMech hood, VelocityMech feeder, VelocityMech indexer, CommandSwerveDrivetrain drivetrain, RangeTable rangeTable, Supplier<Pose2d> poseProvider)
    {
        this.shooter = shooter;
        this.hood = hood;
        this.feeder = feeder;
        this.indexer = indexer;
        this.drivetrain = drivetrain;
        this.poseProvider = poseProvider;
        this.rangeTable = rangeTable;
        currentPose = poseProvider.get();
        //shooter.setTargetSpeed(speed);
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter, hood, feeder, indexer, drivetrain);
    }

    @Override
    public void initialize()
    {
    }

    @Override
    public void execute()
    {
        // Is in shooting area?
        // Get distance and yaw
        // Shooter to distance
        // Drivetrain to yaw

        Pose2d pose = poseProvider.get();
        Translation2d pos = pose.getTranslation();
        Rotation2d heading = pose.getRotation();
        double delta = pos.getDistance(currentPose.getTranslation());

        if (delta >= positionTolerance)
        {
            Translation2d hubPos = Robot.isBlueSide() ? blueHubPos : redHubPos;
            double distance = pos.getDistance(hubPos);

            RangeTable.Range range = rangeTable.getRange(distance);
            double speed = range.speed;
            double hoodPos = range.elevation;

            shooter.setSpeed(speed);
            hood.run(hoodPos);

            double angle = heading.getRadians();
            double kP = 1;
            double turn = -1.0 * angle * kP * MaxAngularRate;
            drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0)
                .withRotationalRate(turn));
            currentPose = pose;
            poseChanged = true;
        }

        boolean atTargetAngle = false;
        // Wait until at setpoint
        // Shoot
        if (shooter.atTarget() && atTargetAngle)
        {
            running = true;
            poseChanged = false;
            indexer.setTargetSpeed(-IndexerConstants.Speed);
            indexer.setSpeed(-IndexerConstants.Speed);
            feeder.setTargetSpeed(-IndexerConstants.FeederSpeed);
            feeder.setSpeed(-IndexerConstants.FeederSpeed);
            drivetrain.applyRequest(() -> brake);
        }
//        SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
    }

    @Override
    public void end(boolean interrupted)
    {
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }
}
