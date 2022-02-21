package team3647.frc2022.autonomous;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import team3647.frc2022.subsystems.Drivetrain;

public class RamseteCommands {
    private final Drivetrain m_drivetrain;
    private final DifferentialDriveKinematics kDriveKinematics;

    public RamseteCommands(Drivetrain drivetrain, DifferentialDriveKinematics driveKinematics) {
        this.m_drivetrain = drivetrain;
        this.kDriveKinematics = driveKinematics;
    }

    public RamseteCommand getStraight() {
        return createDefaulRamseteCommand(Trajectories.straightPath);
    }

    public RamseteCommand getTarmacToBottomLeftBall1() {
        return createDefaulRamseteCommand(Trajectories.tarmacToBottomLeftBall1);
    }

    public RamseteCommand getBottomLeftBall1ToTarmac() {
        return createDefaulRamseteCommand(Trajectories.bottomLeftBall1ToTarmac);
    }

    public RamseteCommand getTarmacToBall2() {
        return createDefaulRamseteCommand(Trajectories.tarmacToBall2);
    }

    public RamseteCommand getBall2ToLoad2() {
        return createDefaulRamseteCommand(Trajectories.ball2ToLoad2);
    }

    public RamseteCommand getLoad2ToShoot() {
        return createDefaulRamseteCommand(Trajectories.load2ToShoot);
    }

    private RamseteCommand createDefaulRamseteCommand(Trajectory trajectory) {
        return new RamseteCommand(
                trajectory,
                m_drivetrain::getPose,
                new RamseteController(),
                kDriveKinematics,
                m_drivetrain::setVelocityLeftRight,
                m_drivetrain);
    }
}
