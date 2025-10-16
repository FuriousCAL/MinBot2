// In src/main/java/frc/robot/Constants/ModuleConstants.java
package frc.robot.Constants;

public final class ModuleConstants {
    // PID constants for the turning motor - very conservative to prevent oscillation
    public static final double kPTurning = 0.5;  // Very low proportional gain to prevent violent oscillation
    public static final double kITurning = 0.0;   // Integral gain (keep at 0 initially)
    public static final double kDTurning = 0.0; // Minimal derivative gain
    
    // Module physical parameters
    public static final double kWheelDiameterMeters = 0.1016; // 4 inches in meters
    public static final double kDriveGearRatio = 6.75; // Example: L2 with 6.75:1 ratio
    public static final double kTurningGearRatio = 12.8; // Example: MK4i L2 with 12.8:1 ratio
    
    // Max speeds
    public static final double kMaxSpeedMetersPerSecond = 4.4; // Example value
    public static final double kMaxTurnSpeedRadPerSec = 2 * Math.PI; // 1 full rotation per second
    public static final double kMaxAccelerationMetersPerSecondSq = 3.0; // Tune this
    
    // Encoder conversion factors
    public static final double kDriveEncoderRot2Meter = (kWheelDiameterMeters * Math.PI) / kDriveGearRatio;
    public static final double kTurningEncoderRot2Rad = (2 * Math.PI) / kTurningGearRatio;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    
    private ModuleConstants() {
        // Private constructor to prevent instantiation
    }
}
