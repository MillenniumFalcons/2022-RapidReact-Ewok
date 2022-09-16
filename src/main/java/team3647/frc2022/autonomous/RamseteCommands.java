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

    public RamseteCommand getBall1ToLoad() {
        return createDefaultRamseteCommand(Trajectories.ball1ToLoad);
    }

    public RamseteCommand getTarmacToBottomLeftBall1() {
        return createDefaultRamseteCommand(Trajectories.tarmacToBottomLeftBall1);
    }

    public RamseteCommand getBottomLeftBall1ToTarmac() {
        return createDefaultRamseteCommand(Trajectories.bottomLeftBall1ToTarmac);
    }

    public RamseteCommand getTarmacToBall2() {
        return createDefaultRamseteCommand(Trajectories.tarmacToBall2);
    }

    public RamseteCommand getBall2ToLoad2() {
        return createDefaultRamseteCommand(Trajectories.ball2ToLoad2);
    }

    public RamseteCommand getLoad2ToShoot() {
        return createDefaultRamseteCommand(Trajectories.load2ToShoot);
    }

    public RamseteCommand getLoad2ToFarShoot() {
        return createDefaultRamseteCommand(Trajectories.load2ToFarShoot);
    }

    public RamseteCommand getTarmacToShootHigh() {
        return createDefaultRamseteCommand(Trajectories.upperTarmacToShootHigh);
    }

    public RamseteCommand getTinyPathToLastHighBall() {
        return createDefaultRamseteCommand(Trajectories.upperTarmacTinyPathToBall);
    }

    public RamseteCommand getUpperBall1ToOtherColorBall1() {
        return createDefaultRamseteCommand(Trajectories.upperBall1ToOtherBall1);
    }

    public RamseteCommand getOtherColorBall1ToTransition() {
        return createDefaultRamseteCommand(Trajectories.otherBall1ToTransition);
    }

    public RamseteCommand getTransitionToOtherColorBall2() {
        return createDefaultRamseteCommand(Trajectories.transitionToOtherBall2);
    }

    public RamseteCommand getOtherColorBall2ToOuttake() {
        return createDefaultRamseteCommand(Trajectories.otherBall2ToHangar);
    }

    public RamseteCommand getUpperTarmactoCurvedBall1() {
        return createDefaultRamseteCommand(Trajectories.upperTarmacToCurvedBall1);
    }

    public RamseteCommand getCurvedBall1ToFlatPoint() {
        return createDefaultRamseteCommand(Trajectories.curvedBall1ToFlatOtherBall);
    }

    private RamseteCommand createDefaultRamseteCommand(Trajectory trajectory) {
        return new RamseteCommand(
                trajectory,
                m_drivetrain::getPose,
                new RamseteController(),
                kDriveKinematics,
                m_drivetrain::setVelocityLeftRight,
                m_drivetrain);
    }
}
