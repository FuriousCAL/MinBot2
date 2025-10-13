package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;

import frc.robot.commands.DriveToAprilTag2Command;
import frc.robot.commands.DriveToHomeCommand;
import frc.robot.commands.SimpleAutonomousCommand;
import frc.robot.commands.VisionAssistedAprilTagCommand;
import frc.robot.constants.AprilTagConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;


public class RobotContainer {
  private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  // Simple deterministic test motions (40% speed/rate)
  private final SwerveRequest.RobotCentric testForward  = new SwerveRequest.RobotCentric()
      .withVelocityX(MaxSpeed * 0.4).withVelocityY(0).withRotationalRate(0);
  private final SwerveRequest.RobotCentric testBackward = new SwerveRequest.RobotCentric()
      .withVelocityX(-MaxSpeed * 0.4).withVelocityY(0).withRotationalRate(0);
  private final SwerveRequest.RobotCentric testRight    = new SwerveRequest.RobotCentric()
      .withVelocityX(0).withVelocityY(-MaxSpeed * 0.4).withRotationalRate(0);
  private final SwerveRequest.RobotCentric testLeft     = new SwerveRequest.RobotCentric()
      .withVelocityX(0).withVelocityY(MaxSpeed * 0.4).withRotationalRate(0);
  private final SwerveRequest.RobotCentric testRotate   = new SwerveRequest.RobotCentric()
      .withVelocityX(0).withVelocityY(0).withRotationalRate(MaxAngularRate * 0.4);

  private final Field2d field = new Field2d();           // Field widget
  private final Telemetry logger;                        // <-- declare only

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  public final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final CommandXboxController joystick = new CommandXboxController(0);

  private SendableChooser<Command> autoChooser;
  // Add a Field-Centric request in parallel to Robot-Centric
  private final SwerveRequest.FieldCentric fieldDrive = new SwerveRequest.FieldCentric()
    .withDeadband(MaxSpeed * 0.1)
    .withRotationalDeadband(MaxAngularRate * 0.1)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Start in field-centric; LB will toggle this
  private boolean isFieldCentric = true;

  // Safe pose (3, 3, 0°) — reuse your constant
  private static final Pose2d SAFE_POSE = AprilTagConstants.HOME_POSITION;

  public RobotContainer() {
    // 1) PathPlanner hooks
    configurePathPlanner();

    // 2) Chooser before adding options
    createAutoChooser();

    // 3) Expose field widget & create telemetry
    SmartDashboard.putData("Field", field);
    logger = new Telemetry(MaxSpeed, field);            // <-- construct once

    // 4) Setup vision pose fusion
    configureVisionFusion();

    // 5) Default teleop drive
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(() -> {
         // Trigger-based speed scaling
        double precision = 1.0 - 0.7 * joystick.getLeftTriggerAxis();  // 1.0 → 0.3
        double turbo     = 1.0 + 0.5 * joystick.getRightTriggerAxis(); // 1.0 → 1.5
        double scale     = MathUtil.clamp(precision * turbo, 0.2, 1.5);

        // Call the concrete request type (avoid the NativeSwerveRequest parent)
        double rightX = joystick.getRightX();
        double rotationRate = -rightX * MaxAngularRate * scale; // Invert for correct rotation direction
        
        // Debug: Print rotation values
        if (Math.abs(rightX) > 0.1) {
          System.out.println("Right stick X: " + rightX + ", Rotation rate: " + rotationRate);
        }
        
        if (isFieldCentric) {
          return fieldDrive
              .withVelocityX(joystick.getLeftY()  * -MaxSpeed         * scale)
              .withVelocityY(joystick.getLeftX()  * -MaxSpeed         * scale)
              .withRotationalRate(rotationRate);
        } else {
          return drive
              .withVelocityX(joystick.getLeftY()  * -MaxSpeed         * scale)
              .withVelocityY(joystick.getLeftX()  * -MaxSpeed         * scale)
              .withRotationalRate(rotationRate);
        }
      })

    );

    // 6) Buttons and SysId
    configureBindings();

    // 7) Add custom auto options
    populateAutoChooser();

    // 8) Feed telemetry (this updates Field2d via Telemetry.telemeterize)
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  /**
   * Configure vision pose fusion to improve PathPlanner accuracy.
   * This is the missing piece that connects VisionSubsystem to CommandSwerveDrivetrain.
   */
  private void configureVisionFusion() {
    // This will be called in periodic() to continuously fuse vision measurements
    System.out.println("[VisionFusion] Vision pose fusion configured");
  }

  /**
   * This should be called periodically from Robot.java to fuse vision measurements.
   */
  public void updateVisionFusion() {
    // Continuously fuse vision poses with drivetrain odometry
    visionSubsystem.getLatestEstimatedPose().ifPresent(estimatedPose -> {
      // Add vision measurement to drivetrain's pose estimator
      drivetrain.addVisionMeasurement(
        estimatedPose.estimatedPose.toPose2d(),
        estimatedPose.timestampSeconds,
        visionSubsystem.getVisionMeasurementStdDevs()
      );
    });
  }

private void configureBindings() {
    // ============================================================================
    // PROFESSIONAL FRC TEAM CONTROLLER LAYOUT (Following 254/971/1678 patterns)
    // ============================================================================
    
    // === CORE DRIVING CONTROLS ===
    // Left Bumper: Toggle Field/Robot Centric (industry standard)
    joystick.leftBumper().onTrue(Commands.runOnce(() -> {
        isFieldCentric = !isFieldCentric;
        System.out.println("Drive mode: " + (isFieldCentric ? "Field" : "Robot") + " centric");
        if (isFieldCentric) {
            drivetrain.seedFieldCentric();
        }
    }));

    // Right Bumper: Brake mode (hold to brake, industry standard)
    joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));

    // A Button: Cancel all commands and return to manual control
    joystick.a().onTrue(Commands.sequence(
        drivetrain.runOnce(() -> {}), // Cancel any active commands
        drivetrain.applyRequest(() -> brake).withTimeout(0.25)
    ));

    // B Button: Point wheels toward left stick direction (precision alignment)
    joystick.b().whileTrue(
        drivetrain.applyRequest(() -> {
            double x = -joystick.getLeftX();
            double y = -joystick.getLeftY();
            double mag = Math.hypot(x, y);
            Rotation2d dir = (mag > 0.10)
                ? new Rotation2d(Math.atan2(y, x))
                : drivetrain.getState().Pose.getRotation();
            return point.withModuleDirection(dir);
        })
    );

    // === VISION-ASSISTED NAVIGATION (Button Combinations) ===
    // X + Y: Drive to AprilTag 2 (Speaker, main scoring position)
    joystick.x().and(joystick.y()).onTrue(new DriveToAprilTag2Command(drivetrain, visionSubsystem));
    
    // X + B: Drive to AprilTag 1 (Blue alliance scoring)
    joystick.x().and(joystick.b()).onTrue(new VisionAssistedAprilTagCommand(drivetrain, visionSubsystem, 1));
    
    // Y + B: Drive to AprilTag 3 (Amp side)
    joystick.y().and(joystick.b()).onTrue(new VisionAssistedAprilTagCommand(drivetrain, visionSubsystem, 3));
    
    // X + A: Drive to AprilTag 4 (Source side)
    joystick.x().and(joystick.a()).onTrue(new VisionAssistedAprilTagCommand(drivetrain, visionSubsystem, 4));

    // === SAFETY AND UTILITY ===
    // Y Button: Return to home position (safe zone)
    joystick.y().onTrue(new DriveToHomeCommand(drivetrain));
    
    // Start Button: Vision system test (drive to AprilTag 2 with simple PID)
    joystick.start().onTrue(new DriveToAprilTag2Command(drivetrain, visionSubsystem));
    
    // Back Button: Emergency home (backup safety)
    joystick.back().onTrue(new DriveToHomeCommand(drivetrain));

    // === D-PAD: QUICK NAVIGATION (Optional) ===
    // D-pad Up: Quick home return
    joystick.povUp().onTrue(new DriveToHomeCommand(drivetrain));
    
    // D-pad Down: Vision test (same as Start)
    joystick.povDown().onTrue(new DriveToAprilTag2Command(drivetrain, visionSubsystem));

    System.out.println("[Controller] Professional FRC team layout loaded");
    System.out.println("  Core: LB=Toggle Mode, RB=Brake, A=Cancel, B=Point, Y=Home");
    System.out.println("  Vision: X+Y=Tag2, X+B=Tag1, Y+B=Tag3, X+A=Tag4");
    System.out.println("  Safety: Start=Vision Test, Back=Emergency Home");
    System.out.println("  Rotation: Left stick = CCW, Right stick = CW");
}


  private void configurePathPlanner() {
    RobotConfig cfg;
    try {
        cfg = RobotConfig.fromGUISettings();
        System.out.println("[PathPlanner] Successfully loaded config: " + cfg.toString());
    } catch (Exception e) {
        System.err.println("[PathPlanner] Failed to load config: " + e.getMessage());
        e.printStackTrace();
        throw new RuntimeException("Failed to load PathPlanner RobotConfig from GUI settings", e);
    }

    AutoBuilder.configure(
        drivetrain::getPose,
        drivetrain::resetPose,
        drivetrain::getRobotRelativeSpeeds,
        (speeds, ffs) -> drivetrain.driveRobotRelative(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0),
            new PIDConstants(5.0, 0.0, 0.0)
        ),
        cfg,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        drivetrain
    );
    
    System.out.println("[PathPlanner] AutoBuilder configured successfully");
}

  private void createAutoChooser() {
    try {
      autoChooser = AutoBuilder.buildAutoChooser();
    } catch (IllegalStateException ex) {
      autoChooser = new SendableChooser<>();
      System.err.println("[Auto] AutoBuilder not configured, using empty chooser: " + ex.getMessage());
    }
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void populateAutoChooser() {
    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Drive Forward 2s", SimpleAutonomousCommand.driveForward(drivetrain, 2.0));
    autoChooser.addOption("Spin In Place 2s", SimpleAutonomousCommand.spinInPlace(drivetrain, 2.0));
    autoChooser.addOption("Square Pattern", SimpleAutonomousCommand.squarePattern(drivetrain));
    autoChooser.addOption("PathPlanner: TestAuto1", AutoBuilder.buildAuto("TestAuto1"));
    autoChooser.addOption(
      "PathPlanner: Simple Test",
      Commands.sequence(
          Commands.print("PathPlanner: Starting simple test"),
          Commands.runOnce(() -> {
              System.out.println("[PathPlanner] Current pose: " + drivetrain.getPose());
              System.out.println("[PathPlanner] Robot config loaded: " + (AutoBuilder.isConfigured() ? "YES" : "NO"));
          }),
          Commands.waitSeconds(1.0),
          Commands.print("PathPlanner: Simple test complete")
      )
  );

  }

  public Command getAutonomousCommand() {
    return autoChooser != null ? autoChooser.getSelected() : Commands.none();
  }
}
