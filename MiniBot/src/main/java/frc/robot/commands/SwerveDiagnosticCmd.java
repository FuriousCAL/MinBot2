package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Diagnostic command for testing swerve drive functionality
 * This command helps identify issues with drive motor inversion and steering
 */
public class SwerveDiagnosticCmd extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private int testPhase = 0;
    private int phaseCounter = 0;
    private static final int PHASE_DURATION = 100; // 2 seconds at 50Hz
    
    public SwerveDiagnosticCmd(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        testPhase = 0;
        phaseCounter = 0;
        SmartDashboard.putString("Swerve Diagnostic", "Starting diagnostic test...");
        System.out.println("=== SWERVE DIAGNOSTIC TEST STARTED ===");
    }

    @Override
    public void execute() {
        phaseCounter++;
        
        switch (testPhase) {
            case 0: // Test individual module drive motors
                testDriveMotors();
                break;
            case 1: // Test individual module steering
                testSteeringMotors();
                break;
            case 2: // Test coordinated movement
                testCoordinatedMovement();
                break;
            case 3: // Test rotation
                testRotation();
                break;
            default:
                testPhase = 0; // Loop back to start
                phaseCounter = 0;
                break;
        }
        
        // Advance to next phase
        if (phaseCounter >= PHASE_DURATION) {
            testPhase++;
            phaseCounter = 0;
            if (testPhase > 3) {
                testPhase = 0; // Loop back to start
            }
        }
        
        // Update dashboard with current test info
        SmartDashboard.putNumber("Diagnostic Phase", testPhase);
        SmartDashboard.putNumber("Phase Progress", (double)phaseCounter / PHASE_DURATION);
    }

    private void testDriveMotors() {
        SmartDashboard.putString("Swerve Diagnostic", "Testing drive motors - all should move forward");
        
        // Create states with forward movement and 0 degree angle
        SwerveModuleState[] states = new SwerveModuleState[4];
        double speed = 0.3; // 30% speed for safety
        
        for (int i = 0; i < 4; i++) {
            states[i] = new SwerveModuleState(speed, new edu.wpi.first.math.geometry.Rotation2d(0));
        }
        
        swerveSubsystem.setModuleStates(states);
        
        System.out.printf("Drive Test - Phase %d: All modules should drive forward at %.1f%% speed%n", 
            phaseCounter, speed * 100);
    }

    private void testSteeringMotors() {
        SmartDashboard.putString("Swerve Diagnostic", "Testing steering motors - wheels should rotate");
        
        // Create states with no drive but rotating steering angle
        SwerveModuleState[] states = new SwerveModuleState[4];
        double angle = (phaseCounter * 3.6) * Math.PI / 180.0; // 3.6 degrees per cycle = full rotation over phase
        
        for (int i = 0; i < 4; i++) {
            states[i] = new SwerveModuleState(0.0, new edu.wpi.first.math.geometry.Rotation2d(angle));
        }
        
        swerveSubsystem.setModuleStates(states);
        
        if (phaseCounter % 25 == 0) { // Print every 0.5 seconds
            System.out.printf("Steering Test - Target angle: %.1f degrees%n", Math.toDegrees(angle));
        }
    }

    private void testCoordinatedMovement() {
        SmartDashboard.putString("Swerve Diagnostic", "Testing coordinated forward movement");
        
        // Test forward movement using the drive method
        double speed = 0.4; // 40% speed
        swerveSubsystem.drive(speed, 0, 0, false); // Forward, no strafe, no rotation, robot-relative
        
        if (phaseCounter % 25 == 0) { // Print every 0.5 seconds
            System.out.printf("Coordinated Test - Forward at %.1f%% speed%n", speed * 100);
        }
    }

    private void testRotation() {
        SmartDashboard.putString("Swerve Diagnostic", "Testing rotation - robot should spin");
        
        // Test rotation
        double rotSpeed = 0.3; // 30% rotation speed
        swerveSubsystem.drive(0, 0, rotSpeed, false); // No translation, just rotation
        
        if (phaseCounter % 25 == 0) { // Print every 0.5 seconds
            System.out.printf("Rotation Test - Rotating at %.1f%% speed%n", rotSpeed * 100);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
        SmartDashboard.putString("Swerve Diagnostic", "Diagnostic test ended");
        System.out.println("=== SWERVE DIAGNOSTIC TEST ENDED ===");
        
        // Print summary
        System.out.println("\n=== DIAGNOSTIC SUMMARY ===");
        System.out.println("1. Drive Motor Test: Check if all wheels moved forward");
        System.out.println("   - If right rear moved backward, drive motor inversion is still needed");
        System.out.println("2. Steering Test: Check if wheels rotated smoothly");
        System.out.println("   - If wheels vibrated instead of rotating, PID tuning is needed");
        System.out.println("3. Coordinated Movement: Check if robot moved forward straight");
        System.out.println("4. Rotation Test: Check if robot rotated in place");
        System.out.println("Check SmartDashboard for detailed module telemetry");
        System.out.println("========================\n");
    }

    @Override
    public boolean isFinished() {
        return false; // Run until manually stopped
    }
}
