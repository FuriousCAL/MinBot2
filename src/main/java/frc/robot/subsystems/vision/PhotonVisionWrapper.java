package frc.robot.vision;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Minimal PhotonVision hookup: returns the FIELD tag pose from layout if the camera currently
 * sees the requested tag ID; otherwise returns Optional.empty(). This lets higher-level code
 * prefer "vision present" behavior without re-deriving the field pose (which is fixed).
 */
public final class PhotonVisionWrapper {
  private PhotonVisionWrapper() {}

  public static Supplier<Optional<Pose2d>> tagPoseSupplier(String cameraName, int tagId) {
    final PhotonCamera cam = new PhotonCamera(cameraName);
    return () -> {
      var res = cam.getLatestResult();
      if (!res.hasTargets()) return Optional.empty();
      for (PhotonTrackedTarget t : res.getTargets()) {
        if (t.getFiducialId() == tagId) {
          // We saw the tag; return its known FIELD pose from layout/constants.
          return frc.robot.constants.AprilTagConstants.getTagPose(tagId);
        }
      }
      return Optional.empty();
    };
  }
}