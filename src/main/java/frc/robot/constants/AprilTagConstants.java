package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import java.util.Optional;

/**
 * Constants for AprilTag-based vision and field positioning.
 * 
 * This class contains all constants related to AprilTag detection,
 * field layout, and standard robot positions on the field.
 */
public final class AprilTagConstants {

    // ==========================================================================
    // FIELD LAYOUT AND GEOMETRY
    // ==========================================================================

    /** 
     * The official field layout for the current competition year.
     * Loaded once and cached for performance.
     */
    public static final AprilTagFieldLayout FIELD_LAYOUT = 
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    /**
     * The geometric center of the playing field.
     * Calculated dynamically from the field layout dimensions.
     */
    public static final Pose2d FIELD_CENTER;

    static {
        double centerX = 0.0;
        double centerY = 0.0;
        
        try {
            centerX = FIELD_LAYOUT.getFieldLength() / 2.0; // meters
            centerY = FIELD_LAYOUT.getFieldWidth() / 2.0;  // meters
        } catch (Exception ignored) {
            // Use default (0,0) if field layout is unavailable
        }
        
        FIELD_CENTER = new Pose2d(centerX, centerY, Rotation2d.kZero);
    }

    // ==========================================================================
    // STANDARD ROBOT POSITIONS
    // ==========================================================================

    /**
     * Safe default position for robot initialization and emergency stops.
     * Located at field coordinates (3.0m, 3.0m) facing forward (0°).
     * 
     * This position should be:
     * - Away from field walls and obstacles
     * - Accessible from multiple directions
     * - Safe for robot initialization
     */
    public static final Pose2d SAFE_POSE = new Pose2d(3.0, 3.0, Rotation2d.fromDegrees(0.0));

    /**
     * Legacy alias for SAFE_POSE to maintain backward compatibility.
     * 
     * @deprecated Use SAFE_POSE instead for clarity
     */
    @Deprecated
    public static final Pose2d HOME_POSITION = SAFE_POSE;

    // ==========================================================================
    // UTILITY METHODS
    // ==========================================================================

    /**
     * Private constructor to prevent instantiation.
     * This is a constants-only utility class.
     **/
    private AprilTagConstants() {
        throw new UnsupportedOperationException("Constants class cannot be instantiated");
    }



    /**
     * Gets the current field layout.
     * 
     * @return The loaded AprilTag field layout
     */
    public static AprilTagFieldLayout getFieldLayout() {
        return FIELD_LAYOUT;
    }

    /**
     * Gets the pose of a specific AprilTag by ID.
     * 
     * @param tagId The ID of the AprilTag to locate
     * @return Optional containing the tag's pose if found, empty otherwise
     */
    public static Optional<Pose2d> getTagPose(int tagId) {
        try {
            Optional<edu.wpi.first.math.geometry.Pose3d> pose3dOpt = FIELD_LAYOUT.getTagPose(tagId);
            
            if (pose3dOpt.isPresent()) {
                edu.wpi.first.math.geometry.Pose3d pose3d = pose3dOpt.get();
                return Optional.of(new Pose2d(
                    pose3d.getX(), 
                    pose3d.getY(), 
                    pose3d.getRotation().toRotation2d()
                ));
            }
            
            return Optional.empty();
        } catch (Exception e) {
            // Log error in production code
            return Optional.empty();
        }
    }

    /**
     * Checks if a given AprilTag ID exists in the current field layout.
     * 
     * @param tagId The ID to check
     * @return true if the tag exists, false otherwise
     */
    public static boolean isValidTagId(int tagId) {
        return getTagPose(tagId).isPresent();
    }
}
