package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class SimpleAutonomousCommand {
    
    public static Command driveForward(CommandSwerveDrivetrain drivetrain, double seconds) {
        return drivetrain.applyRequest(() -> 
            new SwerveRequest.RobotCentric()
                .withVelocityX(1.0) // 1 m/s forward
                .withVelocityY(0.0)
                .withRotationalRate(0.0)
        ).withTimeout(seconds);
    }
    
    public static Command spinInPlace(CommandSwerveDrivetrain drivetrain, double seconds) {
        return drivetrain.applyRequest(() -> 
            new SwerveRequest.RobotCentric()
                .withVelocityX(0.0)
                .withVelocityY(0.0)
                .withRotationalRate(1.0) // 1 rad/s rotation
        ).withTimeout(seconds);
    }
    
    public static Command squarePattern(CommandSwerveDrivetrain drivetrain) {
        return Commands.sequence(
            // Forward
            drivetrain.applyRequest(() -> 
                new SwerveRequest.RobotCentric()
                    .withVelocityX(1.0)
                    .withVelocityY(0.0)
                    .withRotationalRate(0.0)
            ).withTimeout(1.0),
            // Right
            drivetrain.applyRequest(() -> 
                new SwerveRequest.RobotCentric()
                    .withVelocityX(0.0)
                    .withVelocityY(-1.0)
                    .withRotationalRate(0.0)
            ).withTimeout(1.0),
            // Backward
            drivetrain.applyRequest(() -> 
                new SwerveRequest.RobotCentric()
                    .withVelocityX(-1.0)
                    .withVelocityY(0.0)
                    .withRotationalRate(0.0)
            ).withTimeout(1.0),
            // Left
            drivetrain.applyRequest(() -> 
                new SwerveRequest.RobotCentric()
                    .withVelocityX(0.0)
                    .withVelocityY(1.0)
                    .withRotationalRate(0.0)
            ).withTimeout(1.0)
        );
    }
}
