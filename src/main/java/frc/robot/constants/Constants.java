package frc.robot.constants;

/**
 * Master constants class for MinBot2.
 * 
 * This class organizes all robot constants into logical groups.
 * Following FRC best practices, constants are grouped by subsystem or functionality.
 * 
 * Design principles:
 * - All constants are public static final
 * - Values include units in comments
 * - Grouped logically by robot subsystem
 * - Well-documented with purpose and constraints
 */
public final class Constants {

    // ==========================================================================
    // ROBOT PHYSICAL PROPERTIES
    // ==========================================================================
    
    public static final class Robot {
        /** Robot loop period in seconds */
        public static final double LOOP_PERIOD_SECONDS = 0.02; // 50Hz
        
        /** Robot mass in kg (estimate) */
        public static final double MASS_KG = 54.0; // 120 lbs
        
        /** Robot wheelbase width in meters */
        public static final double WHEELBASE_WIDTH_METERS = 0.6096; // 24 inches
        
        /** Robot wheelbase length in meters */
        public static final double WHEELBASE_LENGTH_METERS = 0.6096; // 24 inches
    }

    // ==========================================================================
    // DRIVETRAIN CONSTANTS
    // ==========================================================================
    
    public static final class Drivetrain {
        /** Maximum linear velocity in m/s */
        public static final double MAX_VELOCITY_MPS = 4.5;
        
        /** Maximum angular velocity in rad/s */
        public static final double MAX_ANGULAR_VELOCITY_RADPS = Math.PI * 2.0;
        
        /** Drivetrain voltage compensation */
        public static final double VOLTAGE_COMPENSATION = 12.0; // volts
        
        /** Current limit for drive motors */
        public static final int DRIVE_CURRENT_LIMIT_AMPS = 40;
        
        /** Current limit for azimuth motors */
        public static final int AZIMUTH_CURRENT_LIMIT_AMPS = 30;
        
        /** Deadband for joystick inputs */
        public static final double JOYSTICK_DEADBAND = 0.1;
    }

    // ==========================================================================
    // VISION CONSTANTS
    // ==========================================================================
    
    public static final class Vision {
        /** Primary camera name as configured in PhotonVision dashboard */
        public static final String PRIMARY_CAMERA_NAME = "ArduCam1";
        
        /** Camera height from ground in meters */
        public static final double CAMERA_HEIGHT_METERS = 0.5;
        
        /** Camera pitch angle in degrees (positive = up) */
        public static final double CAMERA_PITCH_DEGREES = 0.0;
        
        /** Camera yaw angle in degrees (positive = left) */
        public static final double CAMERA_YAW_DEGREES = 0.0;
        
        /** Camera roll angle in degrees (positive = clockwise) */
        public static final double CAMERA_ROLL_DEGREES = 0.0;
        
        /** Transform from robot center to camera */
        public static final double CAMERA_FORWARD_OFFSET_METERS = 0.3; // forward from robot center
        public static final double CAMERA_SIDE_OFFSET_METERS = 0.0;    // left from robot center
        
        /** Maximum distance for reliable AprilTag detection in meters */
        public static final double MAX_APRILTAG_DISTANCE_METERS = 5.0;
        
        /** Minimum area for valid AprilTag detection */
        public static final double MIN_TARGET_AREA = 0.1;
        
        /** Maximum ambiguity for pose estimation (lower = more certain) */
        public static final double MAX_POSE_AMBIGUITY = 0.3;
        
        /** Vision measurement standard deviations [x, y, theta] */
        public static final double[] VISION_MEASUREMENT_STDDEVS = {0.5, 0.5, Math.toRadians(30)};
        
        /** Vision processing pipeline timeout in seconds */
        public static final double VISION_TIMEOUT_SECONDS = 0.1;
        
        /** Enable pose estimation from AprilTags */
        public static final boolean ENABLE_POSE_ESTIMATION = true;
        
        /** Enable vision data logging for analysis */
        public static final boolean ENABLE_VISION_LOGGING = true;
    }

    // ==========================================================================
    // AUTONOMOUS CONSTANTS
    // ==========================================================================
    
    public static final class Auto {
        /** Maximum velocity during autonomous in m/s */
        public static final double MAX_AUTO_VELOCITY_MPS = 3.0;
        
        /** Maximum acceleration during autonomous in m/s² */
        public static final double MAX_AUTO_ACCELERATION_MPSPS = 2.0;
        
        /** Position tolerance for autonomous commands in meters */
        public static final double POSITION_TOLERANCE_METERS = 0.05;
        
        /** Angular tolerance for autonomous commands in degrees */
        public static final double ANGULAR_TOLERANCE_DEGREES = 2.0;
        
        /** Timeout for autonomous commands in seconds */
        public static final double COMMAND_TIMEOUT_SECONDS = 10.0;
    }

    // ==========================================================================
    // OPERATOR INTERFACE CONSTANTS
    // ==========================================================================
    
    public static final class OI {
        /** Driver controller port */
        public static final int DRIVER_CONTROLLER_PORT = 0;
        
        /** Operator controller port */
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        
        /** Trigger threshold for button activation */
        public static final double TRIGGER_THRESHOLD = 0.5;
        
        /** Slow mode speed multiplier */
        public static final double SLOW_MODE_MULTIPLIER = 0.3;
    }

    // ==========================================================================
    // FIELD CONSTANTS
    // ==========================================================================
    
    public static final class Field {
        /** Field length in meters */
        public static final double LENGTH_METERS = 16.54; // 54.27 feet
        
        /** Field width in meters */
        public static final double WIDTH_METERS = 8.21; // 26.94 feet
        
        /** AprilTag height from ground in meters */
        public static final double APRILTAG_HEIGHT_METERS = 1.355; // 53.38 inches
    }

    // ==========================================================================
    // LOGGING AND TELEMETRY
    // ==========================================================================
    
    public static final class Logging {
        /** Enable detailed debug logging */
        public static final boolean DEBUG_ENABLED = false;
        
        /** Log file directory path */
        public static final String LOG_DIRECTORY = "/home/lvuser/logs/";
        
        /** Maximum log file size in bytes */
        public static final long MAX_LOG_FILE_SIZE_BYTES = 50 * 1024 * 1024; // 50MB
        
        /** Telemetry update rate in Hz */
        public static final double TELEMETRY_RATE_HZ = 10.0;
    }

    /**
     * Private constructor to prevent instantiation.
     */
    private Constants() {
        throw new UnsupportedOperationException("Constants class cannot be instantiated");
    }
}
