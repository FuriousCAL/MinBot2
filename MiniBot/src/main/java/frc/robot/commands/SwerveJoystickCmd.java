package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveJoystickCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final DoubleSupplier xSpdFunction, ySpdFunction, turningSpdFunction;
    private final BooleanSupplier fieldOrientedFunction;
    private final BooleanSupplier slowModeFunction;
    private final BooleanSupplier turboModeFunction;
    
    // Deadband constant - reduced from 0.05 to 0.02 for better steering sensitivity
    private static final double kDeadband = 0.02;

    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            DoubleSupplier xSpdFunction, DoubleSupplier ySpdFunction, DoubleSupplier turningSpdFunction,
            BooleanSupplier fieldOrientedFunction, BooleanSupplier slowModeFunction, 
            BooleanSupplier turboModeFunction) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.slowModeFunction = slowModeFunction;
        this.turboModeFunction = turboModeFunction;
        
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("SwerveJoystickCmd started!");
    }

    @Override
    public void execute() {
        // Get real-time joystick inputs
        double xSpeed = xSpdFunction.getAsDouble();
        double ySpeed = ySpdFunction.getAsDouble();
        double turningSpeed = turningSpdFunction.getAsDouble();

        // Debug output (throttled to avoid spam)
        if (System.currentTimeMillis() % 1000 < 20) { // Print every ~1 second
            System.out.printf("Joystick Input - X: %.3f, Y: %.3f, Turn: %.3f%n", xSpeed, ySpeed, turningSpeed);
        }

        // Apply deadband to prevent drift
        xSpeed = Math.abs(xSpeed) > kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > kDeadband ? turningSpeed : 0.0;

        // Apply speed scaling based on mode
        double speedMultiplier = getSpeedMultiplier();
        
        xSpeed *= speedMultiplier;
        ySpeed *= speedMultiplier;
        turningSpeed *= speedMultiplier;

        // Convert to actual speeds
        xSpeed *= DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed *= DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed *= DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // Debug output for final speeds
        if (System.currentTimeMillis() % 1000 < 20) { // Print every ~1 second
            System.out.printf("Final Speeds - X: %.3f, Y: %.3f, Turn: %.3f, Field: %s%n", 
                xSpeed, ySpeed, turningSpeed, fieldOrientedFunction.getAsBoolean());
        }

        // Drive the robot
        swerveSubsystem.drive(xSpeed, ySpeed, turningSpeed, fieldOrientedFunction.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
        System.out.println("SwerveJoystickCmd ended!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    /**
     * Determine speed multiplier based on button inputs
     */
    private double getSpeedMultiplier() {
        if (turboModeFunction.getAsBoolean()) {
            return 1.0; // Full speed
        } else if (slowModeFunction.getAsBoolean()) {
            return 0.3; // Slow mode
        } else {
            return 0.7; // Normal speed
        }
    }
}
