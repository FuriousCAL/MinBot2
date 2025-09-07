package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Optional;

public final class AprilTagConstants {

    // SAFE_POSE requested: (3.0, 3.0, 0°)
    public static final Pose2d SAFE_POSE = new Pose2d(3.0, 3.0, Rotation2d.fromDegrees(0));
    // Back-compat alias if other code references HOME_POSITION
    public static final Pose2d HOME_POSITION = SAFE_POSE;

    // Field layout (cached as a static final)
    public static final AprilTagFieldLayout FIELD_LAYOUT =
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

            // ✅ New: the field center, computed from the layout
    public static final Pose2d FIELD_CENTER;
    static {
        double cx = 0.0, cy = 0.0;
        try {
            cx = FIELD_LAYOUT.getFieldLength() / 2.0;
            cy = FIELD_LAYOUT.getFieldWidth()  / 2.0;
        } catch (Exception ignored) { }
        FIELD_CENTER = new Pose2d(cx, cy, Rotation2d.kZero);
    }

    private AprilTagConstants() {} // no instantiation

    /** Accessor if you prefer a method. */
    public static AprilTagFieldLayout getFieldLayout() {
        return FIELD_LAYOUT;
    }

    /** Convenience: Optional<Pose2d> for a tag ID. */
    public static Optional<Pose2d> getTagPose(int tagId) {
        try {
            var pose3dOpt = FIELD_LAYOUT.getTagPose(tagId);
            if (pose3dOpt.isPresent()) {
                var p = pose3dOpt.get().toPose2d();
                return Optional.of(new Pose2d(p.getX(), p.getY(), p.getRotation()));
            }
            return Optional.empty();
        } catch (Exception e) {
            return Optional.empty();
        }
    }
}
