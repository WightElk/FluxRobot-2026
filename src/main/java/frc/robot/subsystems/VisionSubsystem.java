package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;

/**
 * Vision subsystem for AprilTag detection using PhotonVision.
 * Provides target detection and tracking for autonomous alignment.
 */
public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonCamera cameraBack;
    private PhotonPipelineResult latestResult;
    private boolean cameraConnected = true;
    private int disconnectCount = 0;
    private static final int DISCONNECT_THRESHOLD = 50; // ~1 second at 50Hz

    private final PhotonPoseEstimator photonEstimator;
    private Matrix<N3, N1> curStdDevs;
    private final EstimateConsumer estConsumer;

    private AprilTagFieldLayout fieldLayout;

    /**
     * Creates a new VisionSubsystem.
     * @param cameraName Name of the PhotonVision camera (configured in PhotonVision UI)
     */
    public VisionSubsystem(String cameraName, String cameraBackName, AprilTagFieldLayout fieldLayout, EstimateConsumer estConsumer) {
        this.estConsumer = estConsumer;
        this.fieldLayout = fieldLayout;
        this.camera = new PhotonCamera(cameraName);
        this.cameraBack = new PhotonCamera(cameraBackName);
        this.latestResult = new PhotonPipelineResult();
        photonEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.robotToCam1);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        try {
            // Update latest camera result every loop
            var results = camera.getAllUnreadResults();
            if (!results.isEmpty()) {
                // Get the most recent result
                latestResult = results.get(results.size() - 1);
                cameraConnected = true;
                disconnectCount = 0;
            } else {
                // No new results, increment disconnect counter
                disconnectCount++;
                if (disconnectCount > DISCONNECT_THRESHOLD) {
                    cameraConnected = false;
                }
            }

            Optional<EstimatedRobotPose> visionEst = Optional.empty();
            for (var change : results) {
                visionEst = photonEstimator.update(change);
                updateEstimationStdDevs(visionEst, change.getTargets());
    
                // if (Robot.isSimulation()) {
                //     visionEst.ifPresentOrElse(
                //         est -> getSimDebugField()
                //             .getObject("VisionEstimation")
                //             .setPose(est.estimatedPose.toPose2d()),
                //         () -> {
                //             getSimDebugField().getObject("VisionEstimation").setPoses();
                //         });
                // }
    
                visionEst.ifPresent(est -> {
                    // Change our trust in the measurement based on the tags we can see
                    Matrix<N3, N1> estStdDevs = getEstimationStdDevs();
                    estConsumer.accept(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                });
            }            
        } catch (Exception e) {
            // Camera operation failed
            DriverStation.reportError("Vision camera error: " + e.getMessage(), false);
            cameraConnected = false;
            disconnectCount = DISCONNECT_THRESHOLD;
        }

        // Publish telemetry
        SmartDashboard.putBoolean("Connected", cameraConnected);
        SmartDashboard.putBoolean("HasTargets", hasTargets());
        SmartDashboard.putNumber("TargetID", getTargetID());
        SmartDashboard.putNumber("TargetYaw", getTargetYaw());
        SmartDashboard.putNumber("TargetArea", getTargetArea());
    }

    /**
     * Checks if the camera is connected and responding.
     * @return true if camera is connected
     */
    public boolean isCameraConnected() {
        return cameraConnected;
    }

    /**
     * Checks if any AprilTag targets are currently visible.
     * @return true if at least one target is detected
     */
    public boolean hasTargets() {
        return latestResult.hasTargets();
    }

    /**
     * Gets the best (closest/largest) detected AprilTag target.
     * @return Optional containing the best target, or empty if no targets visible
     */
    public Optional<PhotonTrackedTarget> getBestTarget() {
        if (!hasTargets() || !cameraConnected) {
            return Optional.empty();
        }
        return Optional.ofNullable(latestResult.getBestTarget());
    }

    /**
     * Gets the horizontal angle (yaw) to the best target in degrees.
     * Positive values mean target is to the right.
     * @return Yaw angle in degrees, or 0.0 if no target
     */
    public double getTargetYaw() {
        return getBestTarget()
            .map(PhotonTrackedTarget::getYaw)
            .orElse(0.0);
    }

    /**
     * Gets the vertical angle (pitch) to the best target in degrees.
     * Positive values mean target is above camera center.
     * @return Pitch angle in degrees, or 0.0 if no target
     */
    public double getTargetPitch() {
        return getBestTarget()
            .map(PhotonTrackedTarget::getPitch)
            .orElse(0.0);
    }

    /**
     * Gets the target area as percentage of image (0-100).
     * Larger values mean target is closer.
     * @return Target area percentage, or 0.0 if no target
     */
    public double getTargetArea() {
        return getBestTarget()
            .map(PhotonTrackedTarget::getArea)
            .orElse(0.0);
    }

    /**
     * Gets the ID of the best target AprilTag.
     * @return AprilTag ID, or -1 if no target
     */
    public int getTargetID() {
        return getBestTarget()
            .map(PhotonTrackedTarget::getFiducialId)
            .orElse(-1);
    }

    /**
     * Gets the latest pipeline result.
     * Useful for accessing full result data including pose estimates.
     * @return The most recent PhotonPipelineResult
     */
    public PhotonPipelineResult getLatestResult() {
        return latestResult;
    }

       /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.kSingleTagStdDevs;
        }
        else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = VisionConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }
 
    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}
