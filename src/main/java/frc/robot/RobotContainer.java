// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// AprilTag navigation imports
import frc.robot.commands.DriveToAprilTagCommand;
import frc.robot.commands.SimpleAutonomousCommand;
import frc.robot.constants.AprilTagConstants;

/**
 * RobotContainer for CTRE Phoenix 6 swerve.
 */
public class RobotContainer {
    // === Robot/driver config ===
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // desired top speed
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // ~3/4 rot/sec

    // Swerve requests (Phoenix 6 swerve template style)
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // CTRE-generated drivetrain from Swerve Tuner X
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Driver controller
    private final CommandXboxController joystick = new CommandXboxController(0);

    // Auto chooser
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        // === Default driving control mapping ===
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * -MaxSpeed) // Forward is negative stick Y in WPILib
                     .withVelocityY(joystick.getLeftX() * -MaxSpeed)
                     .withRotationalRate(joystick.getRightX() * -MaxAngularRate)
            )
        );

        configureBindings();
        configurePathPlanner();
        buildAutoChooser();

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureBindings() {
        // Test button bindings to verify buttons work
        joystick.x().onTrue(Commands.runOnce(() -> {
            System.out.println("X Button Pressed - Speaker Navigation (Test)");
        }));
        
        joystick.y().onTrue(Commands.runOnce(() -> {
            System.out.println("Y Button Pressed - Amp Navigation (Test)");
        }));
        
        joystick.b().onTrue(Commands.runOnce(() -> {
            System.out.println("B Button Pressed - Source Navigation (Test)");
        }));
        
        joystick.a().onTrue(Commands.runOnce(() -> {
            System.out.println("A Button Pressed - Cancel Navigation (Test)");
        }));

        // Basic brake/point bindings (moved to different buttons)
        joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.leftTrigger().whileTrue(drivetrain.applyRequest(() -> point.withModuleDirection(
            drivetrain.getState().Pose.getRotation())));

        // Zero field-centric heading
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // SysId bindings (as in your original)
        joystick.back().and(joystick.povUp()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.povDown()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.povUp()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.povDown()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    }

    private void configurePathPlanner() {
        // PathPlanner configuration will be added later when methods are available
        System.out.println("PathPlanner configuration - basic setup complete");
    }

    private void buildAutoChooser() {
        SmartDashboard.putData("Auto Chooser", autoChooser);
        autoChooser.setDefaultOption("Do Nothing", Commands.print("Auto: Do Nothing"));
        
        // Add basic autonomous commands using SimpleAutonomousCommand
        autoChooser.addOption("Drive Forward 2s", SimpleAutonomousCommand.driveForward(drivetrain, 2.0));
        autoChooser.addOption("Spin in Place 2s", SimpleAutonomousCommand.spinInPlace(drivetrain, 2.0));
        autoChooser.addOption("Square Pattern", SimpleAutonomousCommand.squarePattern(drivetrain));
        
        System.out.println("Autonomous chooser configured with basic commands");
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
