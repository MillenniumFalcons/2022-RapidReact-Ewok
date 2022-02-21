package team3647.frc2022.autonomous;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import team3647.frc2022.subsystems.Drivetrain;

public class RamseteCommands {
    private final Drivetrain m_drivetrain;
    private final DifferentialDriveKinematics kDriveKinematics;

    public final RamseteCommand straight;
    public final RamseteCommand tarmacToBottomLeftBall1;
    public final RamseteCommand bottomLeftBall1ToTarmac;
    public final RamseteCommand tarmacToBall2;
    public final RamseteCommand ball2ToLoad2;
    public final RamseteCommand load2ToShoot;

    public RamseteCommands(Drivetrain drivetrain, DifferentialDriveKinematics driveKinematics) {
        this.m_drivetrain = drivetrain;
        this.kDriveKinematics = driveKinematics;
        straight =
                new RamseteCommand(
                        Trajectories.straightPath,
                        m_drivetrain::getPose,
                        new RamseteController(),
                        kDriveKinematics,
                        m_drivetrain::setVelocityLeftRight,
                        m_drivetrain);
        tarmacToBottomLeftBall1 =
                new RamseteCommand(
                        Trajectories.tarmacToBottomLeftBall1,
                        m_drivetrain::getPose,
                        new RamseteController(),
                        kDriveKinematics,
                        m_drivetrain::setVelocityLeftRight,
                        m_drivetrain);
        bottomLeftBall1ToTarmac =
                new RamseteCommand(
                        Trajectories.bottomLeftBall1ToTarmac,
                        m_drivetrain::getPose,
                        new RamseteController(),
                        kDriveKinematics,
                        m_drivetrain::setVelocityLeftRight,
                        m_drivetrain);
        tarmacToBall2 =
                new RamseteCommand(
                        Trajectories.tarmacToBall2,
                        m_drivetrain::getPose,
                        new RamseteController(),
                        kDriveKinematics,
                        m_drivetrain::setVelocityLeftRight,
                        m_drivetrain);
        ball2ToLoad2 =
                new RamseteCommand(
                        Trajectories.ball2ToLoad2,
                        m_drivetrain::getPose,
                        new RamseteController(),
                        kDriveKinematics,
                        m_drivetrain::setVelocityLeftRight,
                        m_drivetrain);
        load2ToShoot =
                new RamseteCommand(
                        Trajectories.load2ToShoot,
                        m_drivetrain::getPose,
                        new RamseteController(),
                        kDriveKinematics,
                        m_drivetrain::setVelocityLeftRight,
                        m_drivetrain);
    }
}
