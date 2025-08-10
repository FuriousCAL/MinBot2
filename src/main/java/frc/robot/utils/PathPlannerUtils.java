package frc.robot.utils;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;

/** Utility helpers for PathPlanner to avoid duplication. */
public final class PathPlannerUtils {
    private PathPlannerUtils() {}

    // Centralized path constraints for all navigation commands
    private static final PathConstraints DEFAULT_PATH_CONSTRAINTS = new PathConstraints(
        2.0, // Max velocity (m/s)
        2.0, // Max acceleration (m/s^2)
        Units.degreesToRadians(360), // Max angular velocity (rad/s)
        Units.degreesToRadians(360)  // Max angular acceleration (rad/s^2)
    );

    /** Get default constraints used by simple navigation commands. */
    public static PathConstraints getDefaultPathConstraints() {
        return DEFAULT_PATH_CONSTRAINTS;
    }

    /** Convenience for creating common constraints quickly. */
    public static PathConstraints createPathConstraints(double maxVelocity, double maxAcceleration) {
        return new PathConstraints(
            maxVelocity, 
            maxAcceleration, 
            Units.degreesToRadians(360), 
            Units.degreesToRadians(360)
        );
    }
}
