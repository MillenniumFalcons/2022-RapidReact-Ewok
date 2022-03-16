package team3647.frc2022.autonomous;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    private final Command runFeederAndAccelerate(double offset) {
        return CommandGroupBase.parallel(
                superstructure.runFeeder(() -> 8),
                superstructure.flywheelCommands.variableVelocity(
                        () -> superstructure.getAimedFlywheelSurfaceVel() - offset));
    }

    public Command getLowFive() {
        CommandGroupBase autoIntakeTest =
                CommandGroupBase.parallel(
                        CommandGroupBase.sequence(
                                ramseteCommands.getTarmacToBottomLeftBall1(),
                                new WaitCommand(1.5),
                                ramseteCommands.getBottomLeftBall1ToTarmac(),
                                new WaitCommand(1.2),
                                ramseteCommands.getTarmacToBall2(),
                                new WaitCommand(1),
                                ramseteCommands.getBall2ToLoad2(),
                                new WaitCommand(1),
                                ramseteCommands.getLoad2ToShoot()),
                        superstructure.deployAndRunIntake(() -> 13),
                        CommandGroupBase.sequence(
                                CommandGroupBase.sequence(
                                        runFeederAndAccelerate(0)
                                                .withTimeout(Trajectories.path1Time * 0.95),
                                        superstructure
                                                .autoAccelerateAndShoot(1.2)
                                                .withTimeout(
                                                        Trajectories.path1Time * 0.05
                                                                + Trajectories.path2Time * 0.05
                                                                + 1.5)),
                                CommandGroupBase.sequence(
                                        runFeederAndAccelerate(0)
                                                .withTimeout(Trajectories.path2Time * 0.99),
                                        superstructure
                                                .autoAccelerateAndShoot(1.2)
                                                .withTimeout(
                                                        Trajectories.path2Time * 0.05
                                                                + Trajectories.path3Time * 0.05
                                                                + 1.2)),
                                CommandGroupBase.sequence(
                                        runFeederAndAccelerate(1.2)
                                                .withTimeout(Trajectories.path3Time * 0.95),
                                        superstructure
                                                .autoAccelerateAndShoot(4)
                                                .withTimeout(Trajectories.path4Time * 0.05 + 1)),
                                CommandGroupBase.sequence(
                                        runFeederAndAccelerate(1)
                                                .withTimeout(
                                                        Trajectories.path4Time * 0.95
                                                                + 1
                                                                + Trajectories.path5Time * 0.9)),
                                superstructure.autoAccelerateAndShoot(1.2)));

        return CommandGroupBase.parallel(
                superstructure.disableCompressor(),
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
