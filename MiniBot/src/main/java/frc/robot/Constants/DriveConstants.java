package frc.robot.Constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * Constants related to the swerve drive configuration.
 */
public final class DriveConstants {
    // Driving Parameters
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kPhysicalMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second
    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(10);
    public static final double kWheelBase = Units.inchesToMeters(10);
    
    // Distance between right and left wheels (meters)
    public static final double kDriveBaseRadius = 
        Math.hypot(kTrackWidth / 2.0, kWheelBase / 2.0);
    
    // Swerve Module configuration
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
    public static final double kDriveMotorGearRatio = 1.0 / 6.75; // 6.75:1
    public static final double kTurningMotorGearRatio = 1.0 / 12.8; // 12.8:1
    
    // Encoder conversion factors
    public static final double kDriveEncoderRot2Meter = 
        kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = 
        kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = 
        kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = 
        kTurningEncoderRot2Rad / 60;
    
    // PID constants for turning motor controllers
    public static final double kPTurning = 0.5;
    public static final double kITurning = 0.0;
    public static final double kDTurning = 0.0;
    
    // Teleop drive constants
    public static final double kTeleDriveMaxSpeedMetersPerSecond = 4.8;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3.0;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3.0;
    
    // Gyro port
    public static final int kGyroPort = 0;
    
    // Swerve module offsets (set to 0 for initial testing - adjust these based on your robot's configuration)
    public static final class ModuleOffsets {
        public static final double FRONT_LEFT = 0.0;
        public static final double FRONT_RIGHT = 0.0;
        public static final double REAR_LEFT = 0.0;
        public static final double REAR_RIGHT = 0.0;
    }
    
    // Swerve module CAN IDs (updated with actual hardware configuration)
    public static final class CANIds {
        // Front Left Module
        public static final int FRONT_LEFT_DRIVE = 13;      // Drive motor
        public static final int FRONT_LEFT_TURNING = 42;    // Steering motor
        public static final int FRONT_LEFT_ENCODER = 11;    // CANCoder
        
        // Front Right Module
        public static final int FRONT_RIGHT_DRIVE = 23;     // Drive motor
        public static final int FRONT_RIGHT_TURNING = 22;   // Steering motor
        public static final int FRONT_RIGHT_ENCODER = 21;   // CANCoder
        
        // Rear Right Module
        public static final int REAR_RIGHT_DRIVE = 33;      // Drive motor
        public static final int REAR_RIGHT_TURNING = 32;    // Steering motor
        public static final int REAR_RIGHT_ENCODER = 31;    // CANCoder
        
        // Rear Left Module
        public static final int REAR_LEFT_DRIVE = 43;       // Drive motor
        public static final int REAR_LEFT_TURNING = 12;     // Steering motor
        public static final int REAR_LEFT_ENCODER = 41;     // CANCoder
    }
    
    // Legacy CAN ID constants for compatibility
    public static final int kFrontLeftDriveMotorPort = CANIds.FRONT_LEFT_DRIVE;
    public static final int kFrontLeftTurningMotorPort = CANIds.FRONT_LEFT_TURNING;
    public static final int kFrontRightDriveMotorPort = CANIds.FRONT_RIGHT_DRIVE;
    public static final int kFrontRightTurningMotorPort = CANIds.FRONT_RIGHT_TURNING;
    public static final int kBackLeftDriveMotorPort = CANIds.REAR_LEFT_DRIVE;
    public static final int kBackLeftTurningMotorPort = CANIds.REAR_LEFT_TURNING;
    public static final int kBackRightDriveMotorPort = CANIds.REAR_RIGHT_DRIVE;
    public static final int kBackRightTurningMotorPort = CANIds.REAR_RIGHT_TURNING;
    
    // Swerve module locations from robot center (positive x is forward, positive y is left)
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),  // Front left
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // Front right
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),  // Rear left
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)  // Rear right
    );
    
    // Current limiting for drive motors
    public static final int kDriveCurrentLimit = 40; // Amps
    
    // Max speed while in slow mode (0.0 to 1.0)
    public static final double kSlowModeMultiplier = 0.4;
    
    // Field of view of the camera in radians
    public static final double kCameraFOV = Units.degreesToRadians(70.0);
    
    // Maximum distance for vision updates (meters)
    public static final double kMaxVisionDistance = 5.0;
    
    // Standard deviations for vision measurements (meters, meters, radians)
    public static final double[] kVisionStdDevs = {0.05, 0.05, 0.01};
}
