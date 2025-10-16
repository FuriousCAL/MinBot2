// In src/main/java/frc/robot/subsystems/SwerveModule.java
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX turningMotor;
    private final CANcoder absoluteEncoder;
    private final PIDController turningPidController;
    private final String name;
    
    // For state monitoring
    private double lastPosition = 0;
    private double lastTime = System.currentTimeMillis();
    private double currentVelocity = 0;
    
    // Encoder offsets and reversals
    private final double absoluteEncoderOffsetRad;
    private final boolean absoluteEncoderReversed;
    private final boolean driveMotorReversed;
    
    // Import constants from ModuleConstants for consistency
    private static final double kDriveEncoderRot2Meter = frc.robot.Constants.ModuleConstants.kDriveEncoderRot2Meter;
    private static final double kDriveEncoderRPM2MeterPerSec = frc.robot.Constants.ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    private static final double kPTurning = frc.robot.Constants.ModuleConstants.kPTurning; // Use the lower value (0.01)
    private static final double kITurning = frc.robot.Constants.ModuleConstants.kITurning;
    private static final double kDTurning = frc.robot.Constants.ModuleConstants.kDTurning;
    private static final double kMaxSpeedMetersPerSecond = frc.robot.Constants.ModuleConstants.kMaxSpeedMetersPerSecond;
    private static final double kMaxAccelerationMetersPerSecondSq = frc.robot.Constants.ModuleConstants.kMaxAccelerationMetersPerSecondSq;
    private static final double kMaxTurnSpeedRadPerSec = frc.robot.Constants.ModuleConstants.kMaxTurnSpeedRadPerSec;

    public SwerveModule(int driveMotorId, int turningMotorId, int canCoderId, String name) {
        this(driveMotorId, turningMotorId, canCoderId, name, 0.0, false);
    }
    
    public SwerveModule(int driveMotorId, int turningMotorId, int canCoderId, String name, 
                       double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        this(driveMotorId, turningMotorId, canCoderId, name, absoluteEncoderOffset, absoluteEncoderReversed, false);
    }
    
    public SwerveModule(int driveMotorId, int turningMotorId, int canCoderId, String name, 
                       double absoluteEncoderOffset, boolean absoluteEncoderReversed, boolean driveMotorReversed) {
        this.driveMotor = new TalonFX(driveMotorId);
        this.turningMotor = new TalonFX(turningMotorId);
        this.absoluteEncoder = new CANcoder(canCoderId);
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        this.driveMotorReversed = driveMotorReversed;
        this.turningPidController = new PIDController(kPTurning, kITurning, kDTurning);
        this.name = name;
        
        configureEncoders();
        configurePidControllers();
        configureMotors();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getDriveVelocity(), 
            new Rotation2d(getTurningPosition())
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            getDrivePosition(),
            new Rotation2d(getTurningPosition())
        );
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        // Get current position for optimization
        double currentAngle = getTurningPosition();
        
        // Optimize the state to avoid spinning more than 90 degrees
        SwerveModuleState optimizedState = SwerveModuleState.optimize(
            desiredState, 
            new Rotation2d(currentAngle)
        );

        // Calculate angle error for debugging and deadband
        double angleError = Math.abs(optimizedState.angle.getRadians() - currentAngle);
        
        // TEMPORARY: Disable steering completely to stop vibration
        // Once vibration stops, gradually reduce this deadband
        double turnOutput = 0.0;
        if (angleError > 1.0) { // Very large deadband - essentially disables steering
            turnOutput = turningPidController.calculate(
                currentAngle, 
                optimizedState.angle.getRadians()
            );
        }

        // Calculate drive output with velocity feedforward
        double driveOutput = optimizedState.speedMetersPerSecond / kMaxSpeedMetersPerSecond;

        // Apply motor inversion if needed
        if (driveMotorReversed) {
            driveOutput = -driveOutput;
        }

        // Apply reasonable safety limits
        driveOutput = Math.max(-1.0, Math.min(1.0, driveOutput)); // Simple clamp to [-1, 1]
        turnOutput = Math.max(-0.5, Math.min(0.5, turnOutput));   // More conservative turning power

        // Set the motor outputs
        driveMotor.set(driveOutput);
        turningMotor.set(turnOutput);
        
        // Enhanced logging for debugging
        SmartDashboard.putNumber(name + "/Drive Output", driveOutput);
        SmartDashboard.putNumber(name + "/Turn Output", turnOutput);
        SmartDashboard.putNumber(name + "/Current Angle", Math.toDegrees(currentAngle));
        SmartDashboard.putNumber(name + "/Target Angle", optimizedState.angle.getDegrees());
        SmartDashboard.putNumber(name + "/Angle Error", Math.toDegrees(angleError));
        SmartDashboard.putNumber(name + "/Current", driveMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber(name + "/Temp", driveMotor.getDeviceTemp().getValueAsDouble());
    }

    private double applyDriveLimits(double output) {
        // Limit acceleration
        double maxAccel = kMaxAccelerationMetersPerSecondSq * 0.02; // 20ms loop time
        output = Math.copySign(
            Math.min(Math.abs(output), Math.abs(currentVelocity) + maxAccel),
            output
        );
        
        // Update velocity tracking
        currentVelocity = output;
        return output;
    }

    private double applyTurnLimits(double output) {
        // Limit rotation speed
        return Math.copySign(
            Math.min(Math.abs(output), kMaxTurnSpeedRadPerSec * 0.02), // 20ms loop time
            output
        );
    }

    private void configureEncoders() {
        // Configure the Talon FX integrated encoders
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveMotor.getConfigurator().apply(driveConfig);
        
        TalonFXConfiguration turningConfig = new TalonFXConfiguration();
        turningMotor.getConfigurator().apply(turningConfig);
        
        // Configure CANCoder with basic settings
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        // Use default absolute sensor range and configure offset
        canCoderConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffsetRad / (2 * Math.PI); // Convert to rotations
        absoluteEncoder.getConfigurator().apply(canCoderConfig);
    }

    private void configurePidControllers() {
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        turningPidController.setTolerance(0.01); // 0.01 radian tolerance
    }
    
    private void configureMotors() {
        // Configure drive motor
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Voltage.PeakForwardVoltage = 12.0;
        driveConfig.Voltage.PeakReverseVoltage = -12.0;
        driveMotor.getConfigurator().apply(driveConfig);
        
        // Configure turning motor
        TalonFXConfiguration turningConfig = new TalonFXConfiguration();
        turningConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turningConfig.Voltage.PeakForwardVoltage = 12.0;
        turningConfig.Voltage.PeakReverseVoltage = -12.0;
        turningMotor.getConfigurator().apply(turningConfig);
    }

    private double getTurningPosition() {
        // Get position from CANCoder (absolute encoder)
        double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI; // Convert to radians
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    private double getDrivePosition() {
        // Convert encoder rotations to meters
        return driveMotor.getRotorPosition().getValueAsDouble() * kDriveEncoderRot2Meter;
    }

    private double getDriveVelocity() {
        // Convert encoder velocity to m/s
        return driveMotor.getRotorVelocity().getValueAsDouble() * kDriveEncoderRPM2MeterPerSec;
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
    
    public void resetEncoders() {
        driveMotor.setPosition(0);
        turningMotor.setPosition(getTurningPosition());
    }
    
    // Getter methods for simulation
    public TalonFX getDriveMotor() {
        return driveMotor;
    }
    
    public TalonFX getTurningMotor() {
        return turningMotor;
    }
    
    public CANcoder getAbsoluteEncoder() {
        return absoluteEncoder;
    }
}
