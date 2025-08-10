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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.config.PIDConstants;

import frc.robot.commands.DriveToAprilTagCommand;
import frc.robot.commands.DriveToHomeCommand;
import frc.robot.commands.SimpleAutonomousCommand;
import frc.robot.constants.AprilTagConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
      .withDeadband(MaxSpeed * 0.1)
      .withRotationalDeadband(MaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Field2d field = new Field2d();           // Field widget
  private final Telemetry logger;                        // <-- declare only

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final CommandXboxController joystick = new CommandXboxController(0);

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // 1) PathPlanner hooks
    configurePathPlanner();

    // 2) Chooser before adding options
    createAutoChooser();

    // 3) Expose field widget & create telemetry
    SmartDashboard.putData("Field", field);
    logger = new Telemetry(MaxSpeed, field);            // <-- construct once

    // 4) Default teleop drive
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() ->
            drive.withVelocityX(joystick.getLeftY() * -MaxSpeed)
                 .withVelocityY(joystick.getLeftX() * -MaxSpeed)
                 .withRotationalRate(joystick.getRightX() * -MaxAngularRate)));

    // 5) Buttons and SysId
    configureBindings();

    // 6) Add custom auto options
    populateAutoChooser();

    // 7) Feed telemetry (this updates Field2d via Telemetry.telemeterize)
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  private void configureBindings() {
    joystick.x().onTrue(Commands.runOnce(() -> System.out.println("X Button - Speaker Nav (test)")));
    joystick.y().onTrue(Commands.runOnce(() -> System.out.println("Y Button - Amp Nav (test)")));
    joystick.b().onTrue(Commands.runOnce(() -> System.out.println("B Button - Source Nav (test)")));
    joystick.a().onTrue(Commands.runOnce(() -> System.out.println("A Button - Cancel Nav (test)")));

    joystick.leftBumper().and(joystick.a()).onTrue(new DriveToHomeCommand(drivetrain));

    joystick.rightBumper().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.leftTrigger().whileTrue(
        drivetrain.applyRequest(() -> point.withModuleDirection(drivetrain.getState().Pose.getRotation())));

    // Zero field-centric heading
    joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

    // SysId
    joystick.back().and(joystick.povUp()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.povDown()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.povUp()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.povDown()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
  }

  private void configurePathPlanner() {
    RobotConfig cfg;
    try {
      cfg = RobotConfig.fromGUISettings();
    } catch (Exception e) {
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
    autoChooser.addOption(
        "PathPlanner: Prefaced Path",
        Commands.sequence(
            Commands.print("PathPlanner: Starting preface"),
            SimpleAutonomousCommand.driveForward(drivetrain, 2.0),
            Commands.print("PathPlanner: Preface complete")
        )
    );
    autoChooser.addOption(
        "PathPlanner: Drive to AprilTag 1",
        new DriveToAprilTagCommand(drivetrain, AprilTagConstants.TagIDs.BLUE_SPEAKER_CENTER)
    );
  }

  public Command getAutonomousCommand() {
    return autoChooser != null ? autoChooser.getSelected() : Commands.none();
  }
}
