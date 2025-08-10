package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import frc.robot.subsystems.CommandSwerveDrivetrain;

/** Builds a PathPlanner command at runtime and runs it (no deprecated ProxyCommand). */
public class PathPlannerNavigationCommand extends Command {
    private final Command inner;
    private final String navigationName;
    private final Pose2d targetPose;

    public PathPlannerNavigationCommand(
            CommandSwerveDrivetrain drivetrain,
            Pose2d targetPose,
            PathConstraints constraints,
            String name) {

        this.navigationName = name;
        this.targetPose = targetPose;

        // Defer construction until initialize, with the correct requirement
        this.inner = Commands.defer(
            () -> AutoBuilder.pathfindToPose(targetPose, constraints),
            Set.of(drivetrain)
        );

        addRequirements(drivetrain);
    }

    @Override public void initialize() { 
        System.out.println("PathPlannerNavigation: starting to " + navigationName + " -> " + targetPose);
        inner.initialize(); 
    }
    @Override public void execute() { inner.execute(); }
    @Override public boolean isFinished() { return inner.isFinished(); }
    @Override public void end(boolean interrupted) {
        System.out.println("PathPlannerNavigation: " + (interrupted ? "interrupted en route to " : "arrived at ") + navigationName);
        inner.end(interrupted);
    }
}
