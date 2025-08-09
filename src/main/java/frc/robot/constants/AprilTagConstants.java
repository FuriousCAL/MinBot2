package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Constants for AprilTag field layout and navigation targets
 */
public final class AprilTagConstants {
    
    // Standard FRC field layout
    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    
    // AprilTag IDs for 2024 Crescendo field
    public static final class TagIDs {
        // Blue Alliance
        public static final int BLUE_SOURCE_RIGHT = 1;
        public static final int BLUE_SOURCE_LEFT = 2;
        public static final int BLUE_SPEAKER_OFFSET = 7;
        public static final int BLUE_SPEAKER_CENTER = 8;
        public static final int BLUE_AMP = 6;
        
        // Red Alliance
        public static final int RED_AMP = 5;
        public static final int RED_SPEAKER_CENTER = 3;
        public static final int RED_SPEAKER_OFFSET = 4;
        public static final int RED_SOURCE_LEFT = 9;
        public static final int RED_SOURCE_RIGHT = 10;
        
        // Stage tags
        public static final int RED_STAGE_LEFT = 11;
        public static final int RED_STAGE_RIGHT = 12;
        public static final int RED_STAGE_CENTER = 13;
        public static final int BLUE_STAGE_CENTER = 14;
        public static final int BLUE_STAGE_LEFT = 15;
        public static final int BLUE_STAGE_RIGHT = 16;
    }
    
    // Approach distances and angles for different tag types
    public static final class ApproachConfig {
        // Distance to stop from AprilTag (meters)
        public static final double SPEAKER_APPROACH_DISTANCE = 1.5;
        public static final double AMP_APPROACH_DISTANCE = 1.0;
        public static final double SOURCE_APPROACH_DISTANCE = 1.2;
        public static final double DEFAULT_APPROACH_DISTANCE = 1.0;
        
        // Approach angles (relative to tag normal)
        public static final Rotation2d SPEAKER_APPROACH_ANGLE = Rotation2d.fromDegrees(0); // Straight on
        public static final Rotation2d AMP_APPROACH_ANGLE = Rotation2d.fromDegrees(0);
        public static final Rotation2d SOURCE_APPROACH_ANGLE = Rotation2d.fromDegrees(0);
    }
    
    /**
     * Get the approach pose for a given AprilTag
     * @param tagId The AprilTag ID
     * @return The pose to navigate to when approaching this tag
     */
    public static Pose2d getApproachPose(int tagId) {
        var tagPose = FIELD_LAYOUT.getTagPose(tagId);
        if (tagPose.isEmpty()) {
            return new Pose2d(); // Return origin if tag not found
        }
        
        Pose2d tag = tagPose.get().toPose2d();
        double approachDistance = getApproachDistance(tagId);
        Rotation2d approachAngle = getApproachAngle(tagId);
        
        // Calculate approach position (in front of the tag)
        Transform2d approachTransform = new Transform2d(
            new Translation2d(-approachDistance, 0), // Negative X means in front of tag
            approachAngle
        );
        
        return tag.transformBy(approachTransform);
    }
    
    /**
     * Get the appropriate approach distance for a tag type
     */
    private static double getApproachDistance(int tagId) {
        switch (tagId) {
            case TagIDs.BLUE_SPEAKER_CENTER:
            case TagIDs.BLUE_SPEAKER_OFFSET:
            case TagIDs.RED_SPEAKER_CENTER:
            case TagIDs.RED_SPEAKER_OFFSET:
                return ApproachConfig.SPEAKER_APPROACH_DISTANCE;
                
            case TagIDs.BLUE_AMP:
            case TagIDs.RED_AMP:
                return ApproachConfig.AMP_APPROACH_DISTANCE;
                
            case TagIDs.BLUE_SOURCE_LEFT:
            case TagIDs.BLUE_SOURCE_RIGHT:
            case TagIDs.RED_SOURCE_LEFT:
            case TagIDs.RED_SOURCE_RIGHT:
                return ApproachConfig.SOURCE_APPROACH_DISTANCE;
                
            default:
                return ApproachConfig.DEFAULT_APPROACH_DISTANCE;
        }
    }
    
    /**
     * Get the appropriate approach angle for a tag type
     */
    private static Rotation2d getApproachAngle(int tagId) {
        switch (tagId) {
            case TagIDs.BLUE_SPEAKER_CENTER:
            case TagIDs.BLUE_SPEAKER_OFFSET:
            case TagIDs.RED_SPEAKER_CENTER:
            case TagIDs.RED_SPEAKER_OFFSET:
                return ApproachConfig.SPEAKER_APPROACH_ANGLE;
                
            case TagIDs.BLUE_AMP:
            case TagIDs.RED_AMP:
                return ApproachConfig.AMP_APPROACH_ANGLE;
                
            case TagIDs.BLUE_SOURCE_LEFT:
            case TagIDs.BLUE_SOURCE_RIGHT:
            case TagIDs.RED_SOURCE_LEFT:
            case TagIDs.RED_SOURCE_RIGHT:
                return ApproachConfig.SOURCE_APPROACH_ANGLE;
                
            default:
                return Rotation2d.fromDegrees(0);
        }
    }
    
    /**
     * Get the center field position for canceling navigation
     */
    public static final Pose2d CENTER_FIELD = new Pose2d(
        Units.feetToMeters(27), // Center of field X
        Units.feetToMeters(13.5), // Center of field Y  
        Rotation2d.fromDegrees(0)
    );
}
