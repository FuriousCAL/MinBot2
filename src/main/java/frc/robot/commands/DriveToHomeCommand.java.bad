package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.constants.AprilTagConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utils.PathPlannerUtils;

/**
 * Command to drive the robot back to a fixed "home" position.
 */
public class DriveToHomeCommand extends Command {
    private final PathPlannerNavigationCommand delegate;

    public DriveToHomeCommand(CommandSwerveDrivetrain drivetrain) {
        this.delegate = new PathPlannerNavigationCommand(
            drivetrain, 
            AprilTagConstants.HOME_POSITION, 
            PathPlannerUtils.getDefaultPathConstraints(),
            "Home Position"
        );
        addRequirements(drivetrain);
    }

    @Override public void initialize() { delegate.initialize(); }
    @Override public void execute() { delegate.execute(); }
    @Override public boolean isFinished() { return delegate.isFinished(); }
    @Override public void end(boolean interrupted) { delegate.end(interrupted); }
}
