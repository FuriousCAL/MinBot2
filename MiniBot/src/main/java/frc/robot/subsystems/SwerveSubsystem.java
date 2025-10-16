// SwerveSubsystem.java - Main swerve drive subsystem
package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.wpilibj.Timer;


public class SwerveSubsystem extends SubsystemBase {
    
    // Standard deviations for pose estimation (adjust based on testing)
    private static final double kXStdDev = 0.1; // meters
    private static final double kYStdDev = 0.1; // meters
    private static final double kThetaStdDev = 0.1; // radians

    // Swerve modules (4 modules for typical swerve drive)
    private final SwerveModule[] swerveModules;
    
    // Gyroscope (Pigeon2)
    private final Pigeon2 gyro;
    
    // Kinematics for converting between robot and module speeds
    private final SwerveDriveKinematics kinematics = DriveConstants.kDriveKinematics;
    
    // Odometry for tracking robot position
    private final SwerveDriveOdometry odometry;
    
    // Pose estimator that fuses odometry with vision
    private final SwerveDrivePoseEstimator poseEstimator;
    
    // Field visualization for simulation/dashboard
    private final Field2d field = new Field2d();



    private int loopCounter = 0;

    public SwerveSubsystem() {
        // Initialize swerve modules with correct CAN IDs and proper motor inversions
        // FIXED: Rear modules were inverted incorrectly - rear left should be inverted, rear right should not be
        swerveModules = new SwerveModule[] {
            new SwerveModule(DriveConstants.CANIds.FRONT_LEFT_DRIVE, DriveConstants.CANIds.FRONT_LEFT_TURNING, 
                DriveConstants.CANIds.FRONT_LEFT_ENCODER, "FrontLeft",
                DriveConstants.ModuleOffsets.FRONT_LEFT, false, false),  // Front left: not inverted
            new SwerveModule(DriveConstants.CANIds.FRONT_RIGHT_DRIVE, DriveConstants.CANIds.FRONT_RIGHT_TURNING, 
                DriveConstants.CANIds.FRONT_RIGHT_ENCODER, "FrontRight",
                DriveConstants.ModuleOffsets.FRONT_RIGHT, false, true),  // Front right: drive motor inverted
            new SwerveModule(DriveConstants.CANIds.REAR_LEFT_DRIVE, DriveConstants.CANIds.REAR_LEFT_TURNING, 
                DriveConstants.CANIds.REAR_LEFT_ENCODER, "BackLeft",
                DriveConstants.ModuleOffsets.REAR_LEFT, false, true),    // Rear left: drive motor inverted
            new SwerveModule(DriveConstants.CANIds.REAR_RIGHT_DRIVE, DriveConstants.CANIds.REAR_RIGHT_TURNING, 
                DriveConstants.CANIds.REAR_RIGHT_ENCODER, "BackRight",
                DriveConstants.ModuleOffsets.REAR_RIGHT, false, false)   // Rear right: not inverted
        };

        // Initialize gyro
        gyro = new Pigeon2(DriveConstants.kGyroPort);
        gyro.getConfigurator().apply(new com.ctre.phoenix6.configs.Pigeon2Configuration());
        
        // Reset gyro to zero
        gyro.setYaw(0);
        
        // Initialize odometry and pose estimator with a proper starting position
        // Start at position (3, 2) facing forward - well inside the 17m x 8m field boundaries
        odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, getRotation2d(), getModulePositions(), 
                                          new Pose2d(3, 2, new Rotation2d(0)));
        poseEstimator = new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, getRotation2d(), getModulePositions(), 
                                                    new Pose2d(3, 2, new Rotation2d(0)));
        
        // Set initial robot pose on the field
        field.setRobotPose(new Pose2d(3, 2, new Rotation2d(0)));
        
        // Add field to SmartDashboard with proper initialization
        SmartDashboard.putData("Field", field);
        
        // Force an initial field update
        field.setRobotPose(new Pose2d(3, 2, new Rotation2d(0)));
        
        // Wait for gyro to calibrate
        Timer.delay(0.5);
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public double getHeading() {
        return Math.IEEEremainder(-gyro.getYaw().getValueAsDouble(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
        poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    @Override
    public void periodic() {
        // Cache values
        Rotation2d rotation = getRotation2d();
        SwerveModulePosition[] modulePositions = getModulePositions();
        Pose2d pose = getPose();

        // Update odometry and pose estimator
        odometry.update(rotation, modulePositions);
        poseEstimator.update(rotation, modulePositions);

        // Update field visualization
        field.setRobotPose(pose);

        // Only update essential SmartDashboard values every loop
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("Robot X", pose.getX());
        SmartDashboard.putNumber("Robot Y", pose.getY());
        SmartDashboard.putNumber("Robot Rotation", pose.getRotation().getDegrees());

        // Throttle debug/extra telemetry
        if (loopCounter++ % 10 == 0) { // every 10 loops
            SmartDashboard.putString("Robot Location", pose.getTranslation().toString());
            for (int i = 0; i < swerveModules.length; i++) {
                SwerveModuleState state = swerveModules[i].getState();
                SmartDashboard.putNumber("Module " + i + " Speed", state.speedMetersPerSecond);
                SmartDashboard.putNumber("Module " + i + " Angle", state.angle.getDegrees());
            }
            SmartDashboard.putData("Field", field);
            SmartDashboard.putBoolean("Field Visible", true);
            SmartDashboard.putString("Field Status", "Updated");
            SmartDashboard.putString("Field Robot Pose", pose.toString());
        }
    }



    public void stopModules() {
        for (SwerveModule module : swerveModules) {
            module.stop();
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        for (int i = 0; i < swerveModules.length; i++) {
            swerveModules[i].setDesiredState(desiredStates[i]);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            states[i] = swerveModules[i].getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[swerveModules.length];
        for (int i = 0; i < swerveModules.length; i++) {
            positions[i] = swerveModules[i].getPosition();
        }
        return positions;
    }

    /**
     * Drive the robot with field-relative or robot-relative controls
     * @param xSpeed Speed in X direction (forward/backward)
     * @param ySpeed Speed in Y direction (left/right)  
     * @param rotSpeed Rotational speed
     * @param fieldRelative Whether to use field-relative control
     */
    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
        // Debug output (throttled to avoid spam)
        if (System.currentTimeMillis() % 1000 < 20) { // Print every ~1 second
            System.out.printf("SwerveSubsystem.drive() called - X: %.3f, Y: %.3f, Rot: %.3f, Field: %s%n", 
                xSpeed, ySpeed, rotSpeed, fieldRelative);
        }
        
        ChassisSpeeds chassisSpeeds;
        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rotSpeed, getRotation2d());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        }
        
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        
        // Debug module states
        if (System.currentTimeMillis() % 1000 < 20) { // Print every ~1 second
            for (int i = 0; i < moduleStates.length; i++) {
                System.out.printf("Module %d - Speed: %.3f, Angle: %.1f°%n", 
                    i, moduleStates[i].speedMetersPerSecond, moduleStates[i].angle.getDegrees());
            }
        }
        
        setModuleStates(moduleStates);
    }

    /**
     * Add vision measurement to pose estimator
     * @param visionPose Pose from vision system
     * @param timestamp Timestamp of the measurement
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp);
    }
    
    /**
     * Get current chassis speeds
     */
    public ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }
    
    /**
     * Get the Field2d object for visualization
     */
    public Field2d getField() {
        return field;
    }
    
    public void updatePoseWithVision(VisionSubsystem visionSubsystem) {
        // This method will be implemented when vision is added
    }
    
    public void simulationInit() {
        System.out.println("SwerveSubsystem simulation initialized");
    }

    public void simulationPeriodic() {
        // Update field visualization in simulation
        field.setRobotPose(getPose());
    }
}
