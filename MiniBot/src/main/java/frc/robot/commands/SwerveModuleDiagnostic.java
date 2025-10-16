package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Diagnostic command to test individual swerve modules
 * This helps identify which modules are causing oscillation
 */
public class SwerveModuleDiagnostic extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private int testPhase = 0;
    private int loopCounter = 0;
    private static final int PHASE_DURATION = 100; // loops per phase (2 seconds at 50Hz)
    
    public SwerveModuleDiagnostic(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        testPhase = 0;
        loopCounter = 0;
        System.out.println("Starting Swerve Module Diagnostic Test");
        SmartDashboard.putString("Diagnostic Phase", "Starting");
    }

    @Override
    public void execute() {
        loopCounter++;
        
        // Create desired states for each test phase
        SwerveModuleState[] desiredStates = new SwerveModuleState[4];
        
        // Initialize all modules to stopped
        for (int i = 0; i < 4; i++) {
            desiredStates[i] = new SwerveModuleState(0.0, new Rotation2d(0));
        }
        
        switch (testPhase) {
            case 0: // All modules stopped
                SmartDashboard.putString("Diagnostic Phase", "All Stopped");
                // All already set to stopped
                break;
                
            case 1: // Test front left module only
                SmartDashboard.putString("Diagnostic Phase", "Front Left Only");
                desiredStates[0] = new SwerveModuleState(0.5, new Rotation2d(0)); // 0.5 m/s forward
                break;
                
            case 2: // Test front right module only
                SmartDashboard.putString("Diagnostic Phase", "Front Right Only");
                desiredStates[1] = new SwerveModuleState(0.5, new Rotation2d(0)); // 0.5 m/s forward
                break;
                
            case 3: // Test rear left module only
                SmartDashboard.putString("Diagnostic Phase", "Rear Left Only");
                desiredStates[2] = new SwerveModuleState(0.5, new Rotation2d(0)); // 0.5 m/s forward
                break;
                
            case 4: // Test rear right module only
                SmartDashboard.putString("Diagnostic Phase", "Rear Right Only");
                desiredStates[3] = new SwerveModuleState(0.5, new Rotation2d(0)); // 0.5 m/s forward
                break;
                
            case 5: // Test all modules turning to 45 degrees
                SmartDashboard.putString("Diagnostic Phase", "All Turn 45°");
                for (int i = 0; i < 4; i++) {
                    desiredStates[i] = new SwerveModuleState(0.0, new Rotation2d(Math.PI / 4)); // 45 degrees
                }
                break;
                
            case 6: // Test all modules turning to -45 degrees
                SmartDashboard.putString("Diagnostic Phase", "All Turn -45°");
                for (int i = 0; i < 4; i++) {
                    desiredStates[i] = new SwerveModuleState(0.0, new Rotation2d(-Math.PI / 4)); // -45 degrees
                }
                break;
                
            case 7: // Test all modules back to 0 degrees
                SmartDashboard.putString("Diagnostic Phase", "All Turn 0°");
                for (int i = 0; i < 4; i++) {
                    desiredStates[i] = new SwerveModuleState(0.0, new Rotation2d(0)); // 0 degrees
                }
                break;
                
            default:
                testPhase = 0; // Reset to beginning
                loopCounter = 0;
                break;
        }
        
        // Apply the desired states
        swerveSubsystem.setModuleStates(desiredStates);
        
        // Move to next phase after duration
        if (loopCounter >= PHASE_DURATION) {
            testPhase++;
            loopCounter = 0;
            
            if (testPhase > 7) {
                testPhase = 0; // Loop back to beginning
            }
        }
        
        // Display progress
        SmartDashboard.putNumber("Diagnostic Progress", (double)loopCounter / PHASE_DURATION);
        SmartDashboard.putNumber("Diagnostic Phase Number", testPhase);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
        SmartDashboard.putString("Diagnostic Phase", "Stopped");
        System.out.println("Swerve Module Diagnostic Test Ended");
    }

    @Override
    public boolean isFinished() {
        return false; // Run until manually stopped
    }
}
