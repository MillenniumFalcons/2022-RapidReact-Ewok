package team3647.frc2022.autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import team3647.frc2022.subsystems.Drivetrain;

public class RamseteCommands {
    private final Drivetrain m_drivetrain;
    private final DifferentialDriveKinematics kDriveKinematics;

    public RamseteCommands(Drivetrain drivetrain, DifferentialDriveKinematics driveKinematics) {
        this.m_drivetrain = drivetrain;
        this.kDriveKinematics = driveKinematics;
    }

    public RamseteCommand getFive1() {
        return createDefaultRamseteCommand(PathPlannerTrajectories.five1);
    }

    public RamseteCommand getFive2() {
        return createDefaultRamseteCommand(PathPlannerTrajectories.five2);
    }

    public RamseteCommand getFive3() {
        return createDefaultRamseteCommand(PathPlannerTrajectories.five3);
    }

    public RamseteCommand getFive4() {
        return createDefaultRamseteCommand(PathPlannerTrajectories.five4);
    }

    public RamseteCommand getFive5() {
        return createDefaultRamseteCommand(PathPlannerTrajectories.five5);
    }

    private RamseteCommand createDefaultRamseteCommand(PathPlannerTrajectory trajectory) {
        return new RamseteCommand(
                trajectory,
                m_drivetrain::getPose,
                new RamseteController(),
                kDriveKinematics,
                m_drivetrain::setVelocityLeftRight,
                m_drivetrain);
    }
}
