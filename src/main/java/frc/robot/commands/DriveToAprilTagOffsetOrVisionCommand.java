package frc.robot.commands;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AprilTagConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.PathPlannerUtils;

/** Drives to an offset in front of a tag, using vision pose if available else field layout. */
public class DriveToAprilTagOffsetOrVisionCommand extends Command {
  private final PathPlannerNavigationCommand delegate;

  public DriveToAprilTagOffsetOrVisionCommand(
      CommandSwerveDrivetrain drivetrain,
      int tagId,
      double distanceMeters,
      Supplier<Optional<Pose2d>> visionTagPoseSupplier) {

    Pose2d tagPose = visionTagPoseSupplier.get()
        .orElse(AprilTagConstants.getTagPose(tagId).orElse(AprilTagConstants.FIELD_CENTER));

    Translation2d forward = new Translation2d(distanceMeters, tagPose.getRotation());
    Translation2d targetTranslation = tagPose.getTranslation().plus(forward);
    Rotation2d targetHeading = tagPose.getRotation().plus(Rotation2d.fromDegrees(180.0));
    Pose2d targetPose = new Pose2d(targetTranslation, targetHeading);

    this.delegate = new PathPlannerNavigationCommand(
      drivetrain, targetPose, PathPlannerUtils.getDefaultPathConstraints(), "Tag " + tagId + " offset");
    addRequirements(drivetrain);
  }

  @Override public void initialize() { delegate.initialize(); }
  @Override public void execute() { delegate.execute(); }
  @Override public boolean isFinished() { return delegate.isFinished(); }
  @Override public void end(boolean interrupted) { delegate.end(interrupted); }
}