// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.SwerveModuleDiagnostic;
import frc.robot.commands.SwerveDiagnosticCmd;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(0); // Driver controller port

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    
    // Set default commands
    configureDefaultCommands();
  }

  /**
   * Configure default commands for subsystems
   */
  private void configureDefaultCommands() {
    // Set the swerve subsystem's default command to the teleop drive command
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> -m_driverController.getLeftY(), // Forward/backward (inverted)
        () -> -m_driverController.getLeftX(), // Left/right (inverted)
        () -> -m_driverController.getRightX(), // Rotation (inverted)
        () -> !m_driverController.a().getAsBoolean(), // Field oriented (inverted so default is field-oriented)
        () -> m_driverController.leftBumper().getAsBoolean(), // Slow mode
        () -> m_driverController.rightBumper().getAsBoolean())); // Turbo mode
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    
    // Reset gyro heading to zero
    m_driverController.y().onTrue(new InstantCommand(() -> swerveSubsystem.zeroGyro()));
    
    // Reset robot pose to origin (useful for testing)
    m_driverController.x().onTrue(new InstantCommand(() -> 
        swerveSubsystem.resetOdometry(new edu.wpi.first.math.geometry.Pose2d())));
    
    // Stop all modules (emergency stop)
    m_driverController.start().onTrue(new InstantCommand(() -> swerveSubsystem.stopModules()));
    
    // Run swerve module diagnostic test (hold back button)
    m_driverController.back().whileTrue(new SwerveModuleDiagnostic(swerveSubsystem));
    
    // Run comprehensive swerve diagnostic test (hold left trigger)
    m_driverController.leftTrigger().whileTrue(new SwerveDiagnosticCmd(swerveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  public SwerveSubsystem getSwerveSubsystem() {
    return swerveSubsystem;
  }

  public VisionSubsystem getVisionSubsystem() {
    return visionSubsystem;
  }
}
