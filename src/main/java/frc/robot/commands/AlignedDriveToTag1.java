package frc.robot.commands;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Optional;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.Direction;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Command to drive toward and align with the nearest AprilTag.
 * Uses simple P-controllers for both rotation (yaw) and forward drive (area/distance).
 *
 * Distance calibration: Approximate relationship between distance and target area percentage.
 * Based on typical AprilTag size (6-8 inches) and camera FOV.
 */
public class AlignedDriveToTag1 extends Command {
    private final VisionSubsystem vision;
    private final CommandSwerveDrivetrain drivetrain;

    private AprilTagFieldLayout fieldLayout;
    double length = 0.0;
    double width = 0.0;

    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    // Cached command to avoid lambda creation every execute cycle
    private Command cachedDriveCommand;
    private HolonomicDriveController driveController;
    private VisionConstants.Direction relativePosition = VisionConstants.Direction.Center;

    // Calibration constants for distance-to-area conversion
    // These values need to be calibrated for your specific camera and AprilTag size
    // Formula: area ≈ k / (distance_inches^2) where k is a calibration constant
    private static final double CALIBRATION_CONSTANT = 5000.0; // Adjust based on testing
    private double distanceToTarget = 0;
    private double targetAngle = 0.0;
    private double targetDistance = 0.0;


    /**
     * Creates a new DriveToTag command.
     * @param vision VisionSubsystem for target detection
     * @param drivetrain CommandSwerveDrivetrain for robot movement
     */
    public AlignedDriveToTag1(VisionSubsystem vision, CommandSwerveDrivetrain drivetrain,
        AprilTagFieldLayout fieldLayout, VisionConstants.Direction relativePosition) {
        this.vision = vision;
        this.drivetrain = drivetrain;
        this.fieldLayout = fieldLayout;
        this.relativePosition = relativePosition;

        length = fieldLayout.getFieldLength();
        width = fieldLayout.getFieldWidth();

        driveController = new HolonomicDriveController(
            new PIDController(DriveConstants.Holo_X_kP, DriveConstants.Holo_X_kI, DriveConstants.Holo_X_kD),
            new PIDController(DriveConstants.Holo_Y_kP, DriveConstants.Holo_Y_kI, DriveConstants.Holo_Y_kD),
            new ProfiledPIDController(DriveConstants.Holo_Rot_kP, DriveConstants.Holo_Rot_kI, DriveConstants.Holo_Rot_kD,
                new TrapezoidProfile.Constraints(6.28, 3.14)));
        driveController.setTolerance(DriveConstants.Path_Tolerance);
/*
        switch (relativePosition) {
            case Left:
                // Rotate field layout 90 degrees CCW
                fieldLayout.rotateBy(new Rotation3d(0, 0, Math.toRadians(-90)));
                break;
            case Right:
                // Rotate field layout 90 degrees CW
                fieldLayout.rotateBy(new Rotation3d(0, 0, Math.toRadians(90)));
                break;
            case Center:
            default:
                // No rotation needed
                break;
        }
*/
    
        if (vision != null)
            addRequirements(vision, drivetrain);
    }

    @Override
    public void initialize() {
        cachedDriveCommand = null; // Reset cached command on initialization

        // Initialize SmartDashboard values if not already set
        if (!SmartDashboard.containsKey("TargetDistance_Inches")) {
            SmartDashboard.putNumber("TargetDistance_Inches", VisionConstants.TargetDistance); // Default 30 inches
        }
    }
    
    Pose3d calculatePoseAtTarget(int tagId) {
        Optional<Pose3d> tagPose = fieldLayout.getTagPose(tagId);
        if (tagPose.isEmpty())
            return new Pose3d();

        Pose3d pos = tagPose.get();
        Rotation3d rot = pos.getRotation();
        double angle = rot.getZ() + Math.PI / 2;
        double xDistance = Constants.robotLength / 2.0 + Constants.VisionConstants.distanceToTag;
        Translation3d shift = new Translation3d(xDistance, new Rotation3d(0, 0, angle));

        targetAngle = Math.atan2(VisionConstants.ReefPolesGap, xDistance);
        targetAngle = Math.toRadians(10);

        targetDistance = Math.sqrt(xDistance * xDistance + VisionConstants.ReefPolesGap * VisionConstants.ReefPolesGap);

        if (relativePosition == VisionConstants.Direction.Left) {
            pos.plus(new Transform3d(shift, rot));
        }
        else if (relativePosition == VisionConstants.Direction.Right) {
            targetAngle = -targetAngle;
        }

        Pose3d targetPose = tagPose.get();

        return targetPose;
    }

    /**
     * Converts distance in inches to expected target area percentage.
     * @param distanceInches Distance to target in inches
     * @return Approximate target area percentage
     */
    private double inchesToArea(double distanceInches) {
        if (distanceInches <= 0)
            return 100.0; // Safety check
        return CALIBRATION_CONSTANT / (distanceInches * distanceInches);
    }

    /**
     * Converts target area percentage to approximate distance in inches.
     * @param area Target area percentage
     * @return Approximate distance in inches
     */
    private double areaToInches(double area) {
        if (area <= 0)
            return 1000.0; // Safety check
        return Math.sqrt(CALIBRATION_CONSTANT / area);
    }

    @Override
    public void execute() {
        if (vision == null)
            return;

        if (!vision.hasTargets()) {
            // No target visible - stop moving
            drivetrain.setControl(driveRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
            );
            return;
        }

        // Get target information
        int tagId = vision.getTargetID();
        if (tagId == -1 || fieldLayout == null)
            return;

        Pose3d tagPose = fieldLayout.getTagPose(tagId).orElse(null);
        if (tagPose == null)
            return;

        Translation3d tran = tagPose.getTranslation();
        Rotation3d rot = tagPose.getRotation();
        double yawTag = rot.getZ();

        Rotation3d rotTag = new Rotation3d(0, 0, yawTag);
        rotTag.rotateBy(new Rotation3d(0, 0, Math.toRadians(90)));

        yawTag += Math.toRadians(90);
        //yawTag = clamp(yawTag, Math.toRadians(-180), Math.toRadians(180));

        // Translation3d targetPos = calculatePositionAtTarget(tagId);
        // Rotation3d targetRot = calculateRotationAtTarget(tagId);
        Pose3d targetPose = calculatePoseAtTarget(tagId);

        double yawError = vision.getTargetYaw() - targetAngle;
        double currentArea = vision.getTargetArea();

        SmartDashboard.putNumber("TargetDistance_Inches", targetDistance);

        // Get tunable target distance from SmartDashboard (in inches)
        double targetDistanceInches = SmartDashboard.getNumber("TargetDistance_Inches", VisionConstants.TargetDistance);
        targetDistanceInches = targetDistanceInches / 0.0254;
        double targetArea = inchesToArea(targetDistanceInches);

        // Calculate current estimated distance
        double currentDistanceInches = areaToInches(currentArea);
        distanceToTarget = currentDistanceInches;

        // Publish current distance info to dashboard
        SmartDashboard.putNumber("CurrentDistance_Inches", currentDistanceInches);
        SmartDashboard.putNumber("DistanceError_Inches", currentDistanceInches - targetDistanceInches);

        double areaError = targetArea - currentArea;

        // Calculate rotation speed (align with target)
        double calculatedRotation = -yawError * VisionConstants.ROTATION_P;
        if (Math.abs(yawError) > VisionConstants.ANGLE_TOLERANCE) {
            if (Math.abs(calculatedRotation) < VisionConstants.MIN_ROTATION_SPEED) {
                calculatedRotation = Math.copySign(VisionConstants.MIN_ROTATION_SPEED, calculatedRotation);
            }
        }
        final double rotationSpeed = Math.max(-VisionConstants.MAX_ROTATION_SPEED,
            Math.min(VisionConstants.MAX_ROTATION_SPEED, calculatedRotation));

        // Calculate forward drive speed (approach target)
        // Positive area error means we're too far, need to drive forward
        double calculatedDrive = areaError * VisionConstants.DRIVE_P;

        // Only drive forward if we're reasonably aligned
        if (Math.abs(yawError) > VisionConstants.MAX_YAW_ERROR_FOR_DRIVE) {
            calculatedDrive = 0; // Rotate first, then drive
        }
        else if (Math.abs(areaError) > VisionConstants.AREA_TOLERANCE) {
            if (Math.abs(calculatedDrive) < VisionConstants.MIN_DRIVE_SPEED) {
                calculatedDrive = Math.copySign(VisionConstants.MIN_DRIVE_SPEED, calculatedDrive);
            }
        }
        final double driveSpeed = Math.max(-VisionConstants.MAX_DRIVE_SPEED,
            Math.min(VisionConstants.MAX_DRIVE_SPEED, calculatedDrive));

        // Apply movement (forward X, no strafe Y, rotation)
        // Cache command to avoid creating new lambda every cycle
        if (cachedDriveCommand == null) {
            cachedDriveCommand = drivetrain.applyRequest(() -> driveRequest
                .withVelocityX(driveSpeed)
                .withVelocityY(0)
                .withRotationalRate(rotationSpeed)
            );
        }

//        SmartDashboard.putNumber("distanceToTarget", distanceToTarget);
        SmartDashboard.putNumber("driveSpeed", driveSpeed);
        SmartDashboard.putNumber("rotationSpeed", rotationSpeed);

        // Update the request with new values and execute
        drivetrain.setControl(driveRequest
            .withVelocityX(driveSpeed)
            .withVelocityY(0)
            .withRotationalRate(rotationSpeed)
        );
//        ChassisSpeeds.fromFieldRelativeSpeeds
        Pose2d currentPose = drivetrain.getPosition();
/*
        // Sample the trajectory at 3.4 seconds from the beginning.
        Trajectory.State goal = trajectory.sample(3.4);
        // Get the adjusted speeds. Here, we want the robot to be facing
        // 70 degrees (in the field-relative coordinate system).
        double angle = targetRot.getAngle();
        //angle = Rotation2d.fromDegrees(70.0);

        ChassisSpeeds adjustedSpeeds = driveController.calculate(currentPose, goal, angle);
        drivetrain.setChassisSpeeds(adjustedSpeeds);
*/
    }

    @Override
    public void end(boolean interrupted) {
        // Stop moving when command ends
        drivetrain.setControl(driveRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0)
        );
        cachedDriveCommand = null; // Clear cache
    }

    @Override
    public boolean isFinished() {
        if (vision == null)
            return true;
        // Finish when both aligned and at target distance
        if (!vision.hasTargets())
            return false; // Keep running until we see and reach target, or get interrupted

//        return controller.atReference();

        double targetDistanceInches = SmartDashboard.getNumber("TargetDistance_Inches", VisionConstants.TargetDistance);
        double targetArea = inchesToArea(targetDistanceInches);
        double yawError = Math.abs(vision.getTargetYaw());
        double areaError = Math.abs(targetArea - vision.getTargetArea());

        return yawError < VisionConstants.ANGLE_TOLERANCE && areaError < VisionConstants.AREA_TOLERANCE;
    }
}
