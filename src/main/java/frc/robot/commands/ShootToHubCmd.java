package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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

    private Pose2d currentPose;

    public ShootToHubCmd(VelocityMech2 shooter, PositionMech hood, VelocityMech feeder, VelocityMech indexer, CommandSwerveDrivetrain drivetrain, Supplier<Pose2d> poseProvider)
    {
        this.shooter = shooter;
        this.hood = hood;
        this.feeder = feeder;
        this.indexer = indexer;
        this.drivetrain = drivetrain;
        this.poseProvider = poseProvider;
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

        // Wait until at setpoint
        // Shoot
/*
        if (shooter.atTarget() && drivetrain.atTargetAngle())
        {
            indexer`.shoot();
        }

        // Calculate drivetrain commands from Joystick values
        double forward = -controller.getLeftY() * Constants.Swerve.kMaxLinearSpeed;
        double strafe = -controller.getLeftX() * Constants.Swerve.kMaxLinearSpeed;
        double turn = -controller.getRightX() * Constants.Swerve.kMaxAngularSpeed;

        // Read in relevant data from the Camera
        boolean targetVisible = false;
        double targetYaw = 0.0;
        var results = camera.getAllUnreadResults();
        if (!results.isEmpty()) {
            // Camera processed a new frame since last
            // Get the last one in the list.
            var result = results.get(results.size() - 1);
            if (result.hasTargets()) {
                // At least one AprilTag was seen by the camera
                for (var target : result.getTargets()) {
                    if (target.getFiducialId() == 7) {
                        // Found Tag 7, record its information
                        targetYaw = target.getYaw();
                        targetVisible = true;
                    }
                }
            }
        }

        // Auto-align when requested
        if (controller.getAButton() && targetVisible) {
            // Driver wants auto-alignment to tag 7
            // And, tag 7 is in sight, so we can turn toward it.
            // Override the driver's turn command with an automatic one that turns toward the tag.
            turn = -1.0 * targetYaw * VISION_TURN_kP * Constants.Swerve.kMaxAngularSpeed;
        }

        // Command drivetrain motors based on target speeds
        drivetrain.drive(forward, strafe, turn);

        // Put debug information to the dashboard
        SmartDashboard.putBoolean("Vision Target Visible", targetVisible);
        */
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
