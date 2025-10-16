// In src/main/java/frc/robot/subsystems/SafetyManager.java
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SafetyManager extends SubsystemBase {
    private static SafetyManager instance;
    private final PowerDistribution pdp;
    
    // Safety state
    private boolean isSafetyEnabled = true;
    private boolean isBrownoutProtectionActive = false;
    private double lastVisionUpdateTime = 0;
    private double lastMotorCheckTime = 0;
    
    // Safety constants
    private static final double BROWNOUT_VOLTAGE = 10.0;
    private static final double VISION_DATA_TIMEOUT_SEC = 1.0;
    private static final double MAX_MOTOR_TEMP = 80.0;
    
    private SafetyManager() {
        pdp = new PowerDistribution();
        SmartDashboard.putBoolean("Safety/Enabled", isSafetyEnabled);
    }
    
    public static synchronized SafetyManager getInstance() {
        if (instance == null) {
            instance = new SafetyManager();
        }
        return instance;
    }
    
    @Override
    public void periodic() {
        double currentTime = Timer.getFPGATimestamp();
        
        // Check for brownout conditions
        checkBrownout();
        
        // Periodically check motor temperatures (every 2 seconds)
        if (currentTime - lastMotorCheckTime > 2.0) {
            checkMotorTemperatures();
            lastMotorCheckTime = currentTime;
        }
        
        updateDashboard();
    }
    
    public boolean isSafetyEnabled() {
        return isSafetyEnabled;
    }
    
    public void setSafetyEnabled(boolean enabled) {
        isSafetyEnabled = enabled;
        SmartDashboard.putBoolean("Safety/Enabled", isSafetyEnabled);
    }
    
    public boolean isBrownoutProtectionActive() {
        return isBrownoutProtectionActive;
    }
    
    public void updateVisionTimestamp() {
        lastVisionUpdateTime = Timer.getFPGATimestamp();
    }
    
    public boolean isVisionDataStale() {
        return (Timer.getFPGATimestamp() - lastVisionUpdateTime) > VISION_DATA_TIMEOUT_SEC;
    }
    
    private void checkBrownout() {
        double voltage = pdp.getVoltage();
        if (voltage < BROWNOUT_VOLTAGE && !isBrownoutProtectionActive) {
            isBrownoutProtectionActive = true;
            System.out.println("Brownout protection activated! Voltage: " + voltage + "V");
        } else if (voltage >= (BROWNOUT_VOLTAGE + 0.5) && isBrownoutProtectionActive) {
            isBrownoutProtectionActive = false;
            System.out.println("Brownout protection deactivated. Voltage: " + voltage + "V");
        }
    }
    
    private void checkMotorTemperatures() {
        // Placeholder - implement motor temperature checking when needed
        // This was previously checking SafeMotorController instances
    }
    
    private void updateDashboard() {
        SmartDashboard.putBoolean("Safety/Brownout", isBrownoutProtectionActive);
        SmartDashboard.putNumber("Power/Voltage", pdp.getVoltage());
        SmartDashboard.putNumber("Power/TotalCurrent", pdp.getTotalCurrent());
        SmartDashboard.putNumber("Power/Temperature", pdp.getTemperature());
    }
}