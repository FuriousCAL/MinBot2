package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;

import frc.robot.constants.AprilTagConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Command to drive the robot back to its home position (3,3) on the field
 */
public class DriveToHomeCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private Command pathCommand;
    
    // Path constraints for home navigation
    private static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
        2.0, // Max velocity (m/s)
        2.0, // Max acceleration (m/s^2)
        Units.degreesToRadians(360), // Max angular velocity (rad/s)
        Units.degreesToRadians(360)  // Max angular acceleration (rad/s^2)
    );
    
    /**
     * Create a command to drive to the home position
     * @param drivetrain The swerve drivetrain
     */
    public DriveToHomeCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        System.out.println("DriveToHomeCommand: Driving to home position (3,3)");
        
        // Get the home position
        Pose2d targetPose = AprilTagConstants.HOME_POSITION;
        
        // Get current robot pose
        Pose2d currentPose = drivetrain.getState().Pose;
        
        System.out.println("Current pose: " + currentPose);
        System.out.println("Target pose: " + targetPose);
        
        try {
            // Generate path to home using PathPlanner AutoBuilder
            pathCommand = AutoBuilder.pathfindToPose(
                targetPose,
                PATH_CONSTRAINTS,
                0.0 // Goal end velocity
            );
            
            if (pathCommand != null) {
                pathCommand.initialize();
                System.out.println("Path generated successfully to home position");
            } else {
                System.err.println("Failed to generate path to home position");
            }
        } catch (Exception e) {
            System.err.println("Error generating path to home position: " + e.getMessage());
            pathCommand = null;
        }
    }
    
    @Override
    public void execute() {
        if (pathCommand != null) {
            pathCommand.execute();
        }
    }
    
    @Override
    public boolean isFinished() {
        if (pathCommand == null) {
            return true; // End immediately if no path was generated
        }
        return pathCommand.isFinished();
    }
    
    @Override
    public void end(boolean interrupted) {
        if (pathCommand != null) {
            pathCommand.end(interrupted);
        }
        
        if (interrupted) {
            System.out.println("DriveToHomeCommand: Interrupted");
        } else {
            System.out.println("DriveToHomeCommand: Arrived at home position");
        }
    }
}
