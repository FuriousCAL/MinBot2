package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import java.util.Optional;

/**
 * Constants for AprilTag field layout and navigation targets
 */
public final class AprilTagConstants {
    
    private AprilTagConstants() {}

    /** Load the 2024 Crescendo field layout (used for tag poses). */
    public static AprilTagFieldLayout getFieldLayout() {
        // Uses the WPILib-provided field layout resource packaged with the library
        return AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    }

    /** Field center (example reference) */
    public static final Pose2d FIELD_CENTER = new Pose2d(
        Units.feetToMeters(27), // Center of field X
        Units.feetToMeters(13.5), // Center of field Y  
        Rotation2d.fromDegrees(0)
    );
    
    /** Home position where the robot starts and can return to */
    public static final Pose2d HOME_POSITION = new Pose2d(
        3.0, // 3 meters X
        3.0, // 3 meters Y
        Rotation2d.fromDegrees(0) // Facing forward
    );

    /** Known 2024 Crescendo tag IDs */
    public static final class TagIDs {
        // Blue Alliance
        public static final int BLUE_SOURCE_RIGHT = 1;
        public static final int BLUE_SOURCE_LEFT = 2;
        public static final int BLUE_SPEAKER_CENTER = 8;
        public static final int BLUE_AMP = 6;
        // Red Alliance
        public static final int RED_AMP = 5;
        public static final int RED_SPEAKER_CENTER = 3;
        public static final int RED_SOURCE_LEFT = 10;
        public static final int RED_SOURCE_RIGHT = 9;
    }

    /** Look up a tag's pose by numeric ID using WPILib's layout; empty if not found. */
    public static Optional<Pose2d> getTagPose(int tagId) {
        try {
            var layout = getFieldLayout();
            var pose3d = layout.getTagPose(tagId);
            if (pose3d.isPresent()) {
                var p = pose3d.get().toPose2d();
                return Optional.of(new Pose2d(p.getX(), p.getY(), p.getRotation()));
            }
            return Optional.empty();
        } catch (Exception e) {
            return Optional.empty();
        }
    }
}
