package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;

import frc.robot.constants.AprilTagConstants;
import frc.robot.constants.AprilTagConstants.TagIDs;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Command to drive to a specific AprilTag using PathPlanner
 */
public class DriveToAprilTagCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final int targetTagId;
    private Command pathCommand;
    
    // Path constraints for AprilTag navigation
    private static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
        2.0, // Max velocity (m/s)
        2.0, // Max acceleration (m/s^2)
        Units.degreesToRadians(360), // Max angular velocity (rad/s)
        Units.degreesToRadians(360)  // Max angular acceleration (rad/s^2)
    );
    
    /**
     * Create a command to drive to a specific AprilTag
     * @param drivetrain The swerve drivetrain
     * @param tagId The AprilTag ID to drive to
     */
    public DriveToAprilTagCommand(CommandSwerveDrivetrain drivetrain, int tagId) {
        this.drivetrain = drivetrain;
        this.targetTagId = tagId;
        addRequirements(drivetrain);
    }
    
    /**
     * Create a command to drive to an alliance-specific AprilTag
     * @param drivetrain The swerve drivetrain
     * @param tagType The type of tag (SPEAKER, AMP, SOURCE)
     */
    public static DriveToAprilTagCommand createAllianceSpecific(CommandSwerveDrivetrain drivetrain, TagType tagType) {
        int tagId = getTagIdForAlliance(tagType);
        return new DriveToAprilTagCommand(drivetrain, tagId);
    }
    
    @Override
    public void initialize() {
        System.out.println("DriveToAprilTagCommand: Driving to AprilTag " + targetTagId);
        
        // Get the target pose for this AprilTag
        Pose2d targetPose = AprilTagConstants.getApproachPose(targetTagId);
        
        // Get current robot pose
        Pose2d currentPose = drivetrain.getState().Pose;
        
        try {
            // Generate path to target using PathPlanner AutoBuilder
            pathCommand = AutoBuilder.pathfindToPose(
                targetPose,
                PATH_CONSTRAINTS,
                0.0 // Goal end velocity
            );
            
            if (pathCommand != null) {
                pathCommand.initialize();
                System.out.println("Path generated successfully to tag " + targetTagId);
            } else {
                System.err.println("Failed to generate path to AprilTag " + targetTagId);
            }
        } catch (Exception e) {
            System.err.println("Error generating path to AprilTag " + targetTagId + ": " + e.getMessage());
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
            System.out.println("DriveToAprilTagCommand: Interrupted");
        } else {
            System.out.println("DriveToAprilTagCommand: Completed approach to AprilTag " + targetTagId);
        }
    }
    
    /**
     * Get the appropriate AprilTag ID based on alliance color and tag type
     */
    private static int getTagIdForAlliance(TagType tagType) {
        var alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
        
        switch (tagType) {
            case SPEAKER:
                return isRed ? TagIDs.RED_SPEAKER_CENTER : TagIDs.BLUE_SPEAKER_CENTER;
            case AMP:
                return isRed ? TagIDs.RED_AMP : TagIDs.BLUE_AMP;
            case SOURCE:
                // Choose the closest source tag (for now, just pick one)
                return isRed ? TagIDs.RED_SOURCE_LEFT : TagIDs.BLUE_SOURCE_LEFT;
            default:
                return isRed ? TagIDs.RED_SPEAKER_CENTER : TagIDs.BLUE_SPEAKER_CENTER;
        }
    }
    
    /**
     * Enum for different types of AprilTags
     */
    public enum TagType {
        SPEAKER,
        AMP,
        SOURCE
    }
}
