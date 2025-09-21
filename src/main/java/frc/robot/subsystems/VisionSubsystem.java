package frc.robot.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import frc.robot.constants.AprilTagConstants;
import frc.robot.constants.Constants;

/**
 * Professional VisionSubsystem for MinBot2.
 * 
 * Features:
 * - Real-time AprilTag detection and pose estimation
 * - Integration with swerve drivetrain pose estimation
 * - Health monitoring and diagnostics
 * - Support for both real hardware and simulation
 * - Professional telemetry and logging
 */
public class VisionSubsystem extends SubsystemBase {
    
    // ==========================================================================
    // HARDWARE AND POSE ESTIMATION
    // ==========================================================================
    
    /** Primary PhotonVision camera */
    private final PhotonCamera camera;
    
    /** PhotonVision pose estimator for robot localization */
    private final PhotonPoseEstimator poseEstimator;
    
    /** Transform from robot center to camera */
    private final Transform3d robotToCameraTransform;
    
    // ==========================================================================
    // STATE VARIABLES
    // ==========================================================================
    
    /** Latest pipeline result from camera */
    private PhotonPipelineResult latestResult = new PhotonPipelineResult();
    
    /** Latest estimated robot pose */
    private Optional<EstimatedRobotPose> latestEstimatedPose = Optional.empty();
    
    /** Connection status tracking */
    private boolean cameraConnected = false;
    private double lastResultTimestamp = 0.0;
    private int consecutiveFailures = 0;
    
    /** Performance metrics */
    private double averageLatencyMs = 0.0;
    private int frameCount = 0;
    private final List<Double> recentLatencies = new ArrayList<>();
    
    /**
     * Creates a new VisionSubsystem.
     */
    public VisionSubsystem() {
        // Initialize camera with configured name
        camera = new PhotonCamera(Constants.Vision.PRIMARY_CAMERA_NAME);
        
        // Create robot-to-camera transform from constants
        robotToCameraTransform = new Transform3d(
            new Translation3d(
                Constants.Vision.CAMERA_FORWARD_OFFSET_METERS,
                Constants.Vision.CAMERA_SIDE_OFFSET_METERS,
                Constants.Vision.CAMERA_HEIGHT_METERS
            ),
            new Rotation3d(
                Math.toRadians(Constants.Vision.CAMERA_ROLL_DEGREES),
                Math.toRadians(Constants.Vision.CAMERA_PITCH_DEGREES),
                Math.toRadians(Constants.Vision.CAMERA_YAW_DEGREES)
            )
        );
        
        // Create pose estimator with field layout (updated constructor)
        poseEstimator = new PhotonPoseEstimator(
            AprilTagConstants.FIELD_LAYOUT,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, // Best for accuracy
            robotToCameraTransform
        );
        
        // Configure pose estimator settings
        poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        
        System.out.println("[VisionSubsystem] Initialized with camera: " + Constants.Vision.PRIMARY_CAMERA_NAME);
        System.out.println("[VisionSubsystem] PhotonVision dashboard: http://192.168.86.30:5800/#/camera");
    }
    
    // ==========================================================================
    // PERIODIC AND STATE MANAGEMENT
    // ==========================================================================
    
    @Override
    public void periodic() {
        updateCameraData();
        updatePoseEstimation();
        updateTelemetry();
        monitorHealth();
    }
    
    /**
     * Updates camera data and connection status.
     */
    private void updateCameraData() {
        try {
            // Get latest result from camera
            PhotonPipelineResult result = camera.getLatestResult();
            
            if (result.getTimestampSeconds() != latestResult.getTimestampSeconds()) {
                // New result received
                latestResult = result;
                lastResultTimestamp = Timer.getFPGATimestamp();
                cameraConnected = true;
                consecutiveFailures = 0;
                
                // Track latency (simplified approach - no latency tracking for now)
                // double latency = 0.0; // PhotonVision API changed - will implement later
                // recentLatencies.add(latency);
                // if (recentLatencies.size() > 50) {
                //     recentLatencies.remove(0);
                // }
                
                // Update average latency
                averageLatencyMs = recentLatencies.stream()
                    .mapToDouble(Double::doubleValue)
                    .average()
                    .orElse(0.0);
                
                frameCount++;
            }
        } catch (Exception e) {
            consecutiveFailures++;
            System.err.println("[VisionSubsystem] Error updating camera data: " + e.getMessage());
        }
    }
    
    /**
     * Updates pose estimation from AprilTag detections.
     */
    private void updatePoseEstimation() {
        if (!Constants.Vision.ENABLE_POSE_ESTIMATION || !hasTargets()) {
            latestEstimatedPose = Optional.empty();
            return;
        }
        
        try {
            // Get pose estimate from PhotonVision (pass the latest result)
            Optional<EstimatedRobotPose> poseResult = poseEstimator.update(latestResult);
            
            if (poseResult.isPresent()) {
                EstimatedRobotPose estimate = poseResult.get();
                
                // Validate pose estimate quality
                if (isPoseEstimateValid(estimate)) {
                    latestEstimatedPose = poseResult;
                    
                    if (Constants.Vision.ENABLE_VISION_LOGGING) {
                        logPoseEstimate(estimate);
                    }
                } else {
                    latestEstimatedPose = Optional.empty();
                }
            } else {
                latestEstimatedPose = Optional.empty();
            }
        } catch (Exception e) {
            latestEstimatedPose = Optional.empty();
            System.err.println("[VisionSubsystem] Error updating pose estimation: " + e.getMessage());
        }
    }
    
    // ==========================================================================
    // PUBLIC API METHODS
    // ==========================================================================
    
    /**
     * Gets the latest estimated robot pose from vision.
     * 
     * @return Optional containing estimated pose if available and valid
     */
    public Optional<EstimatedRobotPose> getLatestEstimatedPose() {
        return latestEstimatedPose;
    }
    
    /**
     * Gets the vision measurement standard deviations for pose estimation.
     * 
     * @return Matrix of standard deviations [x, y, theta]
     */
    public Matrix<N3, N1> getVisionMeasurementStdDevs() {
        // Adjust confidence based on distance and number of tags
        double[] stdDevs = Constants.Vision.VISION_MEASUREMENT_STDDEVS.clone();
        
        if (latestEstimatedPose.isPresent()) {
            EstimatedRobotPose estimate = latestEstimatedPose.get();
            int tagCount = estimate.targetsUsed.size();
            
            // More tags = higher confidence (lower standard deviation)
            double tagMultiplier = Math.max(0.5, 1.0 / tagCount);
            stdDevs[0] *= tagMultiplier; // x
            stdDevs[1] *= tagMultiplier; // y
            stdDevs[2] *= tagMultiplier; // theta
        }
        
        return VecBuilder.fill(stdDevs[0], stdDevs[1], stdDevs[2]);
    }
    
    /**
     * Checks if the camera currently sees any AprilTag targets.
     * 
     * @return true if targets are visible
     */
    public boolean hasTargets() {
        return latestResult.hasTargets();
    }
    
    /**
     * Gets the number of currently visible targets.
     * 
     * @return number of visible targets
     */
    public int getTargetCount() {
        return latestResult.hasTargets() ? latestResult.getTargets().size() : 0;
    }
    
    /**
     * Gets all currently visible AprilTag targets.
     * 
     * @return list of visible targets
     */
    public List<PhotonTrackedTarget> getVisibleTargets() {
        return latestResult.hasTargets() ? latestResult.getTargets() : new ArrayList<>();
    }
    
    /**
     * Checks if a specific AprilTag is currently visible.
     * 
     * @param tagId The AprilTag ID to check
     * @return true if the tag is visible
     */
    public boolean isTagVisible(int tagId) {
        return getVisibleTargets().stream()
            .anyMatch(target -> target.getFiducialId() == tagId);
    }
    
    /**
     * Gets the best (lowest ambiguity) currently visible target.
     * 
     * @return Optional containing the best target if any are visible
     */
    public Optional<PhotonTrackedTarget> getBestTarget() {
        if (!hasTargets()) {
            return Optional.empty();
        }
        
        return getVisibleTargets().stream()
            .min((a, b) -> Double.compare(a.getPoseAmbiguity(), b.getPoseAmbiguity()));
    }
    
    /**
     * Checks if the camera is currently connected and receiving data.
     * 
     * @return true if camera is healthy
     */
    public boolean isCameraConnected() {
        return cameraConnected && (Timer.getFPGATimestamp() - lastResultTimestamp < 1.0);
    }
    
    // ==========================================================================
    // VALIDATION AND HEALTH MONITORING
    // ==========================================================================
    
    /**
     * Validates the quality of a pose estimate.
     * 
     * @param estimate The pose estimate to validate
     * @return true if the estimate meets quality standards
     */
    private boolean isPoseEstimateValid(EstimatedRobotPose estimate) {
        if (estimate.targetsUsed.isEmpty()) {
            return false;
        }
        
        // Check ambiguity for single-tag estimates
        if (estimate.targetsUsed.size() == 1) {
            PhotonTrackedTarget target = estimate.targetsUsed.get(0);
            if (target.getPoseAmbiguity() > Constants.Vision.MAX_POSE_AMBIGUITY) {
                return false;
            }
        }
        
        // Check if pose is within reasonable field boundaries
        Pose3d pose = estimate.estimatedPose;
        if (pose.getX() < -1.0 || pose.getX() > Constants.Field.LENGTH_METERS + 1.0 ||
            pose.getY() < -1.0 || pose.getY() > Constants.Field.WIDTH_METERS + 1.0) {
            return false;
        }
        
        // Check for reasonable height (should be near ground level)
        if (Math.abs(pose.getZ()) > 0.5) {
            return false;
        }
        
        return true;
    }
    
    /**
     * Monitors camera health and connection status.
     */
    private void monitorHealth() {
        // Update connection status based on recent data
        if (Timer.getFPGATimestamp() - lastResultTimestamp > 2.0) {
            cameraConnected = false;
        }
        
        // Log warnings for poor connection
        if (consecutiveFailures > 10) {
            System.err.println("[VisionSubsystem] Warning: " + consecutiveFailures + " consecutive camera failures");
        }
    }
    
    // ==========================================================================
    // TELEMETRY AND LOGGING
    // ==========================================================================
    
    /**
     * Updates telemetry data to the dashboard.
     */
    private void updateTelemetry() {
        // Connection status
        SmartDashboard.putBoolean("Vision/Connected", isCameraConnected());
        SmartDashboard.putNumber("Vision/Consecutive Failures", consecutiveFailures);
        
        // Target information
        SmartDashboard.putBoolean("Vision/Has Targets", hasTargets());
        SmartDashboard.putNumber("Vision/Target Count", getTargetCount());
        
        // Performance metrics
        SmartDashboard.putNumber("Vision/Average Latency (ms)", averageLatencyMs);
        SmartDashboard.putNumber("Vision/Frame Count", frameCount);
        
        // Pose estimation
        if (latestEstimatedPose.isPresent()) {
            Pose2d pose = latestEstimatedPose.get().estimatedPose.toPose2d();
            SmartDashboard.putString("Vision/Estimated Pose", 
                String.format("(%.2f, %.2f, %.1f°)", 
                    pose.getX(), 
                    pose.getY(), 
                    pose.getRotation().getDegrees()));
            SmartDashboard.putNumber("Vision/Tags Used", latestEstimatedPose.get().targetsUsed.size());
        } else {
            SmartDashboard.putString("Vision/Estimated Pose", "None");
            SmartDashboard.putNumber("Vision/Tags Used", 0);
        }
        
        // Best target information
        Optional<PhotonTrackedTarget> bestTarget = getBestTarget();
        if (bestTarget.isPresent()) {
            PhotonTrackedTarget target = bestTarget.get();
            SmartDashboard.putNumber("Vision/Best Target ID", target.getFiducialId());
            SmartDashboard.putNumber("Vision/Best Target Ambiguity", target.getPoseAmbiguity());
            SmartDashboard.putNumber("Vision/Best Target Area", target.getArea());
        } else {
            SmartDashboard.putNumber("Vision/Best Target ID", -1);
        }
        
        // PhotonVision dashboard access
        SmartDashboard.putString("Vision/PhotonVision Dashboard", "http://192.168.86.30:5800/#/camera");
    }
    
    /**
     * Logs detailed pose estimate information.
     * 
     * @param estimate The pose estimate to log
     */
    private void logPoseEstimate(EstimatedRobotPose estimate) {
        if (Constants.Logging.DEBUG_ENABLED) {
            System.out.println(String.format(
                "[VisionSubsystem] Pose estimate: (%.3f, %.3f, %.1f°) using %d tags",
                estimate.estimatedPose.getX(),
                estimate.estimatedPose.getY(),
                estimate.estimatedPose.getRotation().getZ() * 180.0 / Math.PI,
                estimate.targetsUsed.size()
            ));
        }
    }
    
    /**
     * Gets diagnostic information about the vision subsystem.
     * 
     * @return formatted diagnostic string
     */
    public String getDiagnostics() {
        return String.format(
            "Vision: Connected=%s, Targets=%d, Latency=%.1fms, Failures=%d",
            isCameraConnected(),
            getTargetCount(),
            averageLatencyMs,
            consecutiveFailures
        );
    }
}
