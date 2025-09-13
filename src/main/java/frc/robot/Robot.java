// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The main Robot class for MinBot2.
 * 
 * This class defines the robot's behavior during different modes (autonomous, teleop, test).
 * It follows the TimedRobot template and implements the command-based programming model.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    /**
     * Robot initialization - runs when the robot is first started up.
     * This is where we create our RobotContainer which holds all subsystems and commands.
     */
    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
    }

    /**
     * Runs periodically during all robot modes.
     * This is where we run the command scheduler and update telemetry.
     */
    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        
        // Update basic robot telemetry
        updateTelemetry();
    }

    /**
     * Updates telemetry data to the dashboard.
     */
    private void updateTelemetry() {
        double yawDeg = m_robotContainer.drivetrain.getPose().getRotation().getDegrees();
        SmartDashboard.putNumber("Robot Yaw (degrees)", yawDeg);
    }

    /**
     * Runs when the robot is disabled.
     */
    @Override
    public void disabledInit() {
        // TODO: Add any disabled initialization logic
    }

    /**
     * Runs periodically when the robot is disabled.
     */
    @Override
    public void disabledPeriodic() {
        // TODO: Add any disabled periodic logic
    }

    /**
     * Runs when exiting disabled mode.
     */
    @Override
    public void disabledExit() {
        // TODO: Add any disabled exit logic
    }

    /**
     * Runs when autonomous mode starts.
     * Gets the selected autonomous command and schedules it.
     */
    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /**
     * Runs periodically during autonomous mode.
     */
    @Override
    public void autonomousPeriodic() {
        // Command scheduler handles autonomous execution
    }

    /**
     * Runs when exiting autonomous mode.
     */
    @Override
    public void autonomousExit() {
        // TODO: Add any autonomous exit logic
    }

    /**
     * Runs when teleop mode starts.
     * Cancels any running autonomous commands.
     */
    @Override
    public void teleopInit() {
        // Cancel autonomous command when entering teleop
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
            m_autonomousCommand = null;
        }
    }

    /**
     * Runs periodically during teleop mode.
     */
    @Override
    public void teleopPeriodic() {
        // Command scheduler handles teleop execution
    }

    /**
     * Runs when exiting teleop mode.
     */
    @Override
    public void teleopExit() {
        // TODO: Add any teleop exit logic
    }

    /**
     * Runs when test mode starts.
     * Cancels all running commands for safety.
     */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * Runs periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        // TODO: Add test mode logic
    }

    /**
     * Runs when exiting test mode.
     */
    @Override
    public void testExit() {
        // TODO: Add any test exit logic
    }

    /**
     * Runs periodically during simulation.
     */
    @Override
    public void simulationPeriodic() {
        // TODO: Add simulation-specific logic
    }
}
