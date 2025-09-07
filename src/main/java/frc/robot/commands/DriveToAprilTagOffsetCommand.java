package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AprilTagConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.PathPlannerUtils;

/**
 * Navigate to a pose that is {@code distanceMeters} in front of the given AprilTag ID,
 * then face the tag (i.e., rotate 180° relative to the tag's facing).
 */
public class DriveToAprilTagOffsetCommand extends Command {
    private final PathPlannerNavigationCommand delegate;

    public DriveToAprilTagOffsetCommand(CommandSwerveDrivetrain drivetrain, int tagId, double distanceMeters) {
        Optional<Pose2d> tagPoseOpt = AprilTagConstants.getTagPose(tagId);
        Pose2d tagPose = tagPoseOpt.orElse(AprilTagConstants.FIELD_CENTER);

        // "Forward of the tag" along its field-facing rotation
        Translation2d forward = new Translation2d(distanceMeters, tagPose.getRotation());
        Translation2d targetTranslation = tagPose.getTranslation().plus(forward);

        // Face the tag (opposite of the tag's facing rotation)
        Rotation2d targetHeading = tagPose.getRotation().plus(Rotation2d.fromDegrees(180.0));

        Pose2d targetPose = new Pose2d(targetTranslation, targetHeading);

        this.delegate = new PathPlannerNavigationCommand(
            drivetrain, targetPose, PathPlannerUtils.getDefaultPathConstraints(),
            "Tag " + tagId + " +" + String.format("%.2f m", distanceMeters));
        addRequirements(drivetrain);
    }

    @Override public void initialize() { delegate.initialize(); }
    @Override public void execute() { delegate.execute(); }
    @Override public boolean isFinished() { return delegate.isFinished(); }
    @Override public void end(boolean interrupted) { delegate.end(interrupted); }
}