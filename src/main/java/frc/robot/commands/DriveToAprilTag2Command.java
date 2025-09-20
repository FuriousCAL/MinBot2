package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.constants.AprilTagConstants;

/**
 * Simple command to drive the robot toward AprilTag 2 using vision data.
 * This demonstrates how to use real PhotonVision data to control the simulation robot.
 */
public class DriveToAprilTag2Command extends Command {
    
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    
    // PID controllers for position control
    private final PIDController xController = new PIDController(2.0, 0.0, 0.0);
    private final PIDController yController = new PIDController(2.0, 0.0, 0.0);
    private final PIDController rotationController = new PIDController(3.0, 0.0, 0.0);
    
    // Target position (AprilTag 2 location)
    private final Pose2d targetPose;
    
    // Tolerance for "arrived"
    private static final double POSITION_TOLERANCE = 0.5; // meters
    private static final double ROTATION_TOLERANCE = Math.toRadians(10); // degrees
    
    public DriveToAprilTag2Command(CommandSwerveDrivetrain drivetrain, VisionSubsystem vision) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        
        // Get AprilTag 2's position from field layout
        this.targetPose = AprilTagConstants.getTagPose(2).orElse(
            new Pose2d(1.0, 0.0, Rotation2d.fromDegrees(0)) // fallback position
        );
        
        // Add requirements
        addRequirements(drivetrain);
    }
    
    @Override
    public void initialize() {
        System.out.println("DriveToAprilTag2Command: Starting to drive toward AprilTag 2");
        System.out.println("Target position: " + targetPose);
        
        // Reset PID controllers
        xController.reset();
        yController.reset();
        rotationController.reset();
    }
    
    @Override
    public void execute() {
        // Get current robot pose
        Pose2d currentPose = drivetrain.getPose();
        
        // Calculate errors
        double xError = targetPose.getX() - currentPose.getX();
        double yError = targetPose.getY() - currentPose.getY();
        double rotationError = targetPose.getRotation().minus(currentPose.getRotation()).getRadians();
        
        // Calculate PID outputs
        double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
        double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
        double rotationSpeed = rotationController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());
        
        // Limit speeds for safety
        xSpeed = Math.max(-1.0, Math.min(1.0, xSpeed));
        ySpeed = Math.max(-1.0, Math.min(1.0, ySpeed));
        rotationSpeed = Math.max(-1.0, Math.min(1.0, rotationSpeed));
        
        // Drive the robot
        drivetrain.driveRobotRelative(
            new edu.wpi.first.math.kinematics.ChassisSpeeds(xSpeed, ySpeed, rotationSpeed)
        );
        
        // Print debug info
        System.out.printf("DriveToAprilTag2: Current=(%.2f, %.2f, %.1f°), Target=(%.2f, %.2f, %.1f°), Errors=(%.2f, %.2f, %.1f°)\n",
            currentPose.getX(), currentPose.getY(), Math.toDegrees(currentPose.getRotation().getRadians()),
            targetPose.getX(), targetPose.getY(), Math.toDegrees(targetPose.getRotation().getRadians()),
            xError, yError, Math.toDegrees(rotationError)
        );
    }
    
    @Override
    public boolean isFinished() {
        Pose2d currentPose = drivetrain.getPose();
        
        // Check if we're close enough to the target
        double xError = Math.abs(targetPose.getX() - currentPose.getX());
        double yError = Math.abs(targetPose.getY() - currentPose.getY());
        double rotationError = Math.abs(targetPose.getRotation().minus(currentPose.getRotation()).getRadians());
        
        boolean atPosition = xError < POSITION_TOLERANCE && yError < POSITION_TOLERANCE;
        boolean atRotation = rotationError < ROTATION_TOLERANCE;
        
        if (atPosition && atRotation) {
            System.out.println("DriveToAprilTag2Command: Arrived at AprilTag 2!");
            return true;
        }
        
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        // Stop the robot
        drivetrain.driveRobotRelative(new edu.wpi.first.math.kinematics.ChassisSpeeds(0, 0, 0));
        
        if (interrupted) {
            System.out.println("DriveToAprilTag2Command: Interrupted");
        } else {
            System.out.println("DriveToAprilTag2Command: Completed successfully");
        }
    }
}
