package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;

import frc.robot.constants.AprilTagConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.PathPlannerUtils;

/** Command to pathfind to a specific AprilTag pose using the WPILib layout. */
public class DriveToAprilTagCommand extends Command {
    private final PathPlannerNavigationCommand delegate;

    public DriveToAprilTagCommand(CommandSwerveDrivetrain drivetrain, int tagId) {
        Optional<Pose2d> target = AprilTagConstants.getTagPose(tagId);
        Pose2d pose = target.orElse(AprilTagConstants.FIELD_CENTER);
        this.delegate = new PathPlannerNavigationCommand(
            drivetrain, pose, PathPlannerUtils.getDefaultPathConstraints(), "AprilTag " + tagId);
        addRequirements(drivetrain);
    }

    @Override public void initialize() { delegate.initialize(); }
    @Override public void execute() { delegate.execute(); }
    @Override public boolean isFinished() { return delegate.isFinished(); }
    @Override public void end(boolean interrupted) { delegate.end(interrupted); }
}
