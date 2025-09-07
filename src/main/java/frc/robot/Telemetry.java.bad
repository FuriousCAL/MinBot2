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
    }
}
