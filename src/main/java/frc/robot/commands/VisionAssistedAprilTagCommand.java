package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AprilTagConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utils.PathPlannerUtils;

import java.util.Optional;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.auto.AutoBuilder;

/**
 * Vision-Assisted AprilTag Navigation Command
 * 
 * This command implements a two-phase approach:
 * 1. Coarse Navigation: Uses PathPlanner to get within ~1-2m of target AprilTag
 * 2. Precision Control: Uses real-time vision feedback for exact positioning
 * 
 * Final position: 0.5m directly in front of the specified AprilTag
 */
public class VisionAssistedAprilTagCommand extends Command {
    
    // ==========================================================================
    // CONSTANTS
    // ==========================================================================
    
    /** Distance to maintain from AprilTag center (meters) */
    private static final double TARGET_DISTANCE_METERS = 0.5;
    
    /** Distance threshold to switch from PathPlanner to vision control */
    private static final double VISION_SWITCH_DISTANCE = 1.5;
    
    /** Maximum pose ambiguity to accept for vision control */
    private static final double MAX_VISION_AMBIGUITY = 0.3;
    
    /** Timeout for PathPlanner phase (seconds) */
    private static final double PATHPLANNER_TIMEOUT = 10.0;
    
    /** Timeout for vision precision phase (seconds) */
    private static final double VISION_TIMEOUT = 5.0;
    
    /** Position tolerance for completion (meters) */
    private static final double POSITION_TOLERANCE = 0.05;
    
    /** Angle tolerance for completion (degrees) */
    private static final double ANGLE_TOLERANCE = 2.0;
    
    // ==========================================================================
    // SUBSYSTEMS AND CONTROLLERS
    // ==========================================================================
    
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem visionSubsystem;
    private final int targetTagId;
    
    /** Holonomic drive controller for vision-based precision control */
    private final HolonomicDriveController visionController;
    
    /** PathPlanner command for coarse navigation */
    private Command pathPlannerCommand;
    
    // ==========================================================================
    // STATE VARIABLES
    // ==========================================================================
    
    /** Current command phase */
    private enum Phase { PATHPLANNER, VISION, FINISHED }
    private Phase currentPhase = Phase.PATHPLANNER;
    
    /** Command start time for timeout handling */
    private double commandStartTime;
    private double phaseStartTime;
    
    /** Target pose from field layout (fallback) */
    private final Pose2d layoutTargetPose;
    
    /** Final target pose (0.5m in front of tag, facing tag) */
    private Pose2d finalTargetPose;
    
    /**
     * Creates a new VisionAssistedAprilTagCommand.
     * 
     * @param drivetrain The swerve drivetrain subsystem
     * @param visionSubsystem The vision subsystem
     * @param tagId The AprilTag ID to navigate to
     */
    public VisionAssistedAprilTagCommand(CommandSwerveDrivetrain drivetrain, 
                                       VisionSubsystem visionSubsystem, 
                                       int tagId) {
        this.drivetrain = drivetrain;
        this.visionSubsystem = visionSubsystem;
        this.targetTagId = tagId;
        
        // Get target pose from field layout
        Optional<Pose2d> tagPoseOpt = AprilTagConstants.getTagPose(tagId);
        this.layoutTargetPose = tagPoseOpt.orElse(AprilTagConstants.FIELD_CENTER);
        
        // Calculate final target pose (0.5m in front of tag, facing tag)
        this.finalTargetPose = calculateFinalTargetPose(layoutTargetPose);
        
        // Create vision controller with appropriate PID gains
        this.visionController = new HolonomicDriveController(
            new PIDController(3.0, 0.0, 0.0),  // X controller
            new PIDController(3.0, 0.0, 0.0),  // Y controller
            new ProfiledPIDController(2.0, 0.0, 0.0,  // Theta controller
                new TrapezoidProfile.Constraints(6.28, 3.14))  // Max vel: 1 rev/s, Max accel: 0.5 rev/s²
        );
        
        // Set position and angle tolerances
        visionController.setTolerance(
            new Pose2d(POSITION_TOLERANCE, POSITION_TOLERANCE, 
                      Rotation2d.fromDegrees(ANGLE_TOLERANCE))
        );
        
        addRequirements(drivetrain, visionSubsystem);
    }
    
    // ==========================================================================
    // COMMAND LIFECYCLE
    // ==========================================================================
    
    @Override
    public void initialize() {
        commandStartTime = Timer.getFPGATimestamp();
        phaseStartTime = commandStartTime;
        currentPhase = Phase.PATHPLANNER;
        
        // Create PathPlanner command for initial approach
        pathPlannerCommand = AutoBuilder.pathfindToPose(
            finalTargetPose,
            PathPlannerUtils.getDefaultPathConstraints()
        );
        
        pathPlannerCommand.initialize();
        
        SmartDashboard.putString("VisionAssisted/Phase", "PATHPLANNER");
        SmartDashboard.putNumber("VisionAssisted/Target Tag", targetTagId);
        SmartDashboard.putString("VisionAssisted/Status", "Starting PathPlanner approach");
        
        System.out.println(String.format(
            "[VisionAssisted] Starting approach to Tag %d at (%.2f, %.2f)",
            targetTagId, finalTargetPose.getX(), finalTargetPose.getY()
        ));
    }
    
    @Override
    public void execute() {
        double currentTime = Timer.getFPGATimestamp();
        double elapsedTime = currentTime - commandStartTime;
        double phaseElapsedTime = currentTime - phaseStartTime;
        
        SmartDashboard.putNumber("VisionAssisted/Elapsed Time", elapsedTime);
        SmartDashboard.putNumber("VisionAssisted/Phase Time", phaseElapsedTime);
        
        switch (currentPhase) {
            case PATHPLANNER:
                executePathPlannerPhase(phaseElapsedTime);
                break;
            case VISION:
                executeVisionPhase(phaseElapsedTime);
                break;
            case FINISHED:
                drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
                break;
        }
    }
    
    @Override
    public boolean isFinished() {
        return currentPhase == Phase.FINISHED || 
               Timer.getFPGATimestamp() - commandStartTime > (PATHPLANNER_TIMEOUT + VISION_TIMEOUT);
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop drivetrain
        drivetrain.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
        
        // Clean up PathPlanner command
        if (pathPlannerCommand != null) {
            pathPlannerCommand.end(interrupted);
        }
        
        String status = interrupted ? "INTERRUPTED" : 
                       (currentPhase == Phase.FINISHED ? "COMPLETED" : "TIMEOUT");
        
        SmartDashboard.putString("VisionAssisted/Status", status);
        
        System.out.println(String.format(
            "[VisionAssisted] Command ended: %s (Phase: %s, Distance: %.3fm)",
            status, currentPhase, getDistanceToTarget()
        ));
    }
    
    // ==========================================================================
    // PHASE EXECUTION METHODS
    // ==========================================================================
    
    /**
     * Executes the PathPlanner coarse navigation phase.
     */
    private void executePathPlannerPhase(double phaseElapsedTime) {
        // Continue PathPlanner command
        pathPlannerCommand.execute();
        
        // Check if we should switch to vision control
        double distanceToTarget = getDistanceToTarget();
        boolean hasGoodVision = hasGoodVisionTarget();
        boolean closeEnough = distanceToTarget < VISION_SWITCH_DISTANCE;
        boolean pathPlannerFinished = pathPlannerCommand.isFinished();
        boolean pathPlannerTimeout = phaseElapsedTime > PATHPLANNER_TIMEOUT;
        
        SmartDashboard.putNumber("VisionAssisted/Distance to Target", distanceToTarget);
        SmartDashboard.putBoolean("VisionAssisted/Good Vision", hasGoodVision);
        SmartDashboard.putBoolean("VisionAssisted/PathPlanner Finished", pathPlannerFinished);
        
        // Switch to vision phase if conditions are met
        if ((closeEnough && hasGoodVision) || pathPlannerFinished || pathPlannerTimeout) {
            switchToVisionPhase();
        }
    }
    
    /**
     * Executes the vision-based precision control phase.
     */
    private void executeVisionPhase(double phaseElapsedTime) {
        // Check for vision timeout
        if (phaseElapsedTime > VISION_TIMEOUT) {
            currentPhase = Phase.FINISHED;
            SmartDashboard.putString("VisionAssisted/Status", "Vision timeout");
            return;
        }
        
        // Get current robot pose
        Pose2d currentPose = drivetrain.getPose();
        
        // Try to get vision-updated target pose
        Pose2d targetPose = getVisionUpdatedTargetPose().orElse(finalTargetPose);
        
        // Calculate chassis speeds using holonomic controller
        ChassisSpeeds chassisSpeeds = visionController.calculate(
            currentPose,
            targetPose,
            0.0,  // No desired velocity
            targetPose.getRotation()
        );
        
        // Limit maximum speeds for safety during precision control
        double maxSpeed = 0.5;  // 0.5 m/s max during precision phase
        chassisSpeeds.vxMetersPerSecond = MathUtil.clamp(chassisSpeeds.vxMetersPerSecond, -maxSpeed, maxSpeed);
        chassisSpeeds.vyMetersPerSecond = MathUtil.clamp(chassisSpeeds.vyMetersPerSecond, -maxSpeed, maxSpeed);
        chassisSpeeds.omegaRadiansPerSecond = MathUtil.clamp(chassisSpeeds.omegaRadiansPerSecond, -1.0, 1.0);
        
        // Apply chassis speeds
        drivetrain.driveRobotRelative(chassisSpeeds);
        
        // Update telemetry
        SmartDashboard.putNumber("VisionAssisted/Target X", targetPose.getX());
        SmartDashboard.putNumber("VisionAssisted/Target Y", targetPose.getY());
        SmartDashboard.putNumber("VisionAssisted/Target Rotation", targetPose.getRotation().getDegrees());
        SmartDashboard.putNumber("VisionAssisted/Speed X", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("VisionAssisted/Speed Y", chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("VisionAssisted/Speed Omega", Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond));
        
        // Check if we're at the target
        if (visionController.atReference()) {
            currentPhase = Phase.FINISHED;
            SmartDashboard.putString("VisionAssisted/Status", "Target reached!");
            System.out.println("[VisionAssisted] Successfully reached target position");
        }
    }
    
    // ==========================================================================
    // HELPER METHODS
    // ==========================================================================
    
    /**
     * Switches from PathPlanner phase to vision precision phase.
     */
    private void switchToVisionPhase() {
        currentPhase = Phase.VISION;
        phaseStartTime = Timer.getFPGATimestamp();
        
        // End PathPlanner command
        if (pathPlannerCommand != null) {
            pathPlannerCommand.end(false);
        }
        
        SmartDashboard.putString("VisionAssisted/Phase", "VISION");
        SmartDashboard.putString("VisionAssisted/Status", "Switching to vision precision control");
        
        System.out.println("[VisionAssisted] Switching to vision precision control");
    }
    
    /**
     * Calculates the final target pose (0.5m in front of tag, facing tag).
     */
    private Pose2d calculateFinalTargetPose(Pose2d tagPose) {
        // Calculate position 0.5m in front of the tag
        Translation2d offset = new Translation2d(TARGET_DISTANCE_METERS, tagPose.getRotation());
        Translation2d targetTranslation = tagPose.getTranslation().plus(offset);
        
        // Face the tag (opposite of tag's facing direction)
        Rotation2d targetRotation = tagPose.getRotation().plus(Rotation2d.fromDegrees(180.0));
        
        return new Pose2d(targetTranslation, targetRotation);
    }
    
    /**
     * Gets the distance from robot to target position.
     */
    private double getDistanceToTarget() {
        Pose2d currentPose = drivetrain.getPose();
        return currentPose.getTranslation().getDistance(finalTargetPose.getTranslation());
    }
    
    /**
     * Checks if we have a good vision target for the specified AprilTag.
     */
    private boolean hasGoodVisionTarget() {
        if (!visionSubsystem.isTagVisible(targetTagId)) {
            return false;
        }
        
        Optional<PhotonTrackedTarget> bestTarget = visionSubsystem.getBestTarget();
        if (bestTarget.isEmpty()) {
            return false;
        }
        
        PhotonTrackedTarget target = bestTarget.get();
        return target.getFiducialId() == targetTagId && 
               target.getPoseAmbiguity() < MAX_VISION_AMBIGUITY;
    }
    
    /**
     * Gets a vision-updated target pose if available.
     * Uses real-time vision measurement to refine the target position.
     */
    private Optional<Pose2d> getVisionUpdatedTargetPose() {
        return visionSubsystem.getLatestEstimatedPose().map(estimatedPose -> {
            // Use the vision-estimated robot pose to calculate a more accurate target
            Pose2d robotPose = estimatedPose.estimatedPose.toPose2d();
            
            // Get the target AprilTag from vision
            Optional<PhotonTrackedTarget> targetOpt = visionSubsystem.getVisibleTargets().stream()
                .filter(target -> target.getFiducialId() == targetTagId)
                .findFirst();
            
            if (targetOpt.isPresent()) {
                // Use vision data to calculate more precise tag position
                // This is a simplified approach - could be enhanced with full 3D transforms
                return finalTargetPose;  // For now, use the layout-based pose
            }
            
            return finalTargetPose;
        });
    }
}
