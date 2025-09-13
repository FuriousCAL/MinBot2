package frc.robot;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;   // <— NEW
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
/**
 * Telemetry class for publishing swerve drive data to NetworkTables and SmartDashboard.
 * This class handles the publishing of the robot's pose, module states, and module positions.
 */

public class Telemetry {
    private final double m_maxSpeed;
    private final NetworkTable m_table;
    private final StructPublisher<Pose2d> m_posePub;
    private final StructArrayPublisher<SwerveModuleState> m_moduleStatesPub;
    private final StructArrayPublisher<SwerveModulePosition> m_modulePositionsPub;
    private final Field2d m_field;                     // <— NEW

    public Telemetry(double maxSpeed, Field2d field) { // <— CHANGED SIGNATURE
        m_maxSpeed = maxSpeed;
        m_field = field;                               // <— NEW
        m_table = NetworkTableInstance.getDefault().getTable("Swerve");
        m_posePub = m_table.getStructTopic("Pose", Pose2d.struct).publish();
        m_moduleStatesPub = m_table.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();
        m_modulePositionsPub = m_table.getStructArrayTopic("ModulePositions", SwerveModulePosition.struct).publish();
    }

    public void telemeterize(SwerveDriveState state) {
        if (state == null) return;
        m_posePub.set(state.Pose);
        m_moduleStatesPub.set(state.ModuleStates);
        m_modulePositionsPub.set(state.ModulePositions);
        if (m_field != null) {
            m_field.setRobotPose(state.Pose);          // <— THIS DRAWS THE ROBOT
        }
        // Per-module angle debug to SmartDashboard
        String[] names = { "FL", "FR", "BL", "BR" };
        for (int i = 0; i < 4; ++i) {
            double desiredDeg  = state.ModuleTargets[i].angle.getDegrees();
            double measuredDeg = state.ModuleStates[i].angle.getDegrees();
            double deltaDeg    = MathUtil.inputModulus(desiredDeg - measuredDeg, -180.0, 180.0);

            SmartDashboard.putNumber("Swerve/" + names[i] + "/DesiredDeg",  desiredDeg);
            SmartDashboard.putNumber("Swerve/" + names[i] + "/MeasuredDeg", measuredDeg);
            SmartDashboard.putNumber("Swerve/" + names[i] + "/DeltaDeg",    deltaDeg);
        }

        }
    
}
