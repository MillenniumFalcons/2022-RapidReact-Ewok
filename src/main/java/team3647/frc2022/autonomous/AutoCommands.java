package team3647.frc2022.autonomous;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3647.frc2022.subsystems.Drivetrain;
import team3647.frc2022.subsystems.Superstructure;

public class AutoCommands {
    private final Drivetrain drivetrain;
    private final Superstructure superstructure;
    private final RamseteCommands ramseteCommands;

    public AutoCommands(
            Drivetrain drivetrain,
            DifferentialDriveKinematics driveKinematics,
            Superstructure superstructure) {
        this.drivetrain = drivetrain;
        this.superstructure = superstructure;
        ramseteCommands = new RamseteCommands(drivetrain, driveKinematics);
    }

    private final Command runFeederAndAccelerate() {
        return CommandGroupBase.parallel(
                superstructure.runFeeder(() -> 8),
                superstructure.flywheelCommands.variableVelocity(
                        superstructure::getAimedFlywheelSurfaceVel));
    }

    public Command getLowFive() {
        CommandGroupBase autoIntakeTest =
                CommandGroupBase.parallel(
                        CommandGroupBase.sequence(
                                ramseteCommands.getTarmacToBottomLeftBall1(),
                                new WaitCommand(1.5),
                                ramseteCommands.getBottomLeftBall1ToTarmac(),
                                new WaitCommand(1),
                                ramseteCommands.getTarmacToBall2(),
                                new WaitCommand(2),
                                ramseteCommands.getBall2ToLoad2(),
                                ramseteCommands.getLoad2ToShoot()),
                        superstructure.deployAndRunIntake(() -> 8),
                        CommandGroupBase.sequence(
                                CommandGroupBase.sequence(
                                        runFeederAndAccelerate()
                                                .withTimeout(Trajectories.path1Time * 0.7),
                                        superstructure
                                                .autoAccelerateAndShoot()
                                                .withTimeout(
                                                        Trajectories.path1Time * 0.3
                                                                + Trajectories.path2Time * 0.3
                                                                + 1.5)),
                                CommandGroupBase.sequence(
                                        runFeederAndAccelerate()
                                                .withTimeout(Trajectories.path2Time * 0.4),
                                        superstructure
                                                .autoAccelerateAndShoot()
                                                .withTimeout(
                                                        Trajectories.path2Time * 0.3
                                                                + Trajectories.path3Time * 0.3
                                                                + 1)),
                                CommandGroupBase.sequence(
                                        runFeederAndAccelerate()
                                                .withTimeout(Trajectories.path3Time * 0.7 + 0.5),
                                        superstructure
                                                .autoAccelerateAndShoot()
                                                .withTimeout(Trajectories.path4Time * 0.2 + 1.5)),
                                CommandGroupBase.sequence(
                                        runFeederAndAccelerate()
                                                .withTimeout(
                                                        Trajectories.path4Time * 0.8
                                                                + Trajectories.path5Time * 0.7)),
                                new PrintCommand("Started shooting"),
                                superstructure.autoAccelerateAndShoot()));

        return CommandGroupBase.parallel(
                superstructure.turretCommands.motionMagic(0).andThen(superstructure.aimTurret()),
                autoIntakeTest);
    }

    public Command getHighTwo() {
        return new InstantCommand();
        // return ramseteCommands
        //         .getTarmacToHighBall()
        //         .deadlineWith(intakeFromGround)
        //         .andThen(ramseteCommands.getHighBallToShoot().deadlineWith(accelerateFlywheel))
        //         .andThen(autoShoot);
    }
}
