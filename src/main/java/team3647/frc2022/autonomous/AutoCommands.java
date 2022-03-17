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

    public Command lowFiveQuestionable() {
        Command turretSequence =
                superstructure.turretCommands.motionMagic(0).andThen(superstructure.aimTurret());
        Command drivetrainSequence =
                CommandGroupBase.sequence(
                        ramseteCommands.getTarmacToBottomLeftBall1(),
                        ramseteCommands.getBottomLeftBall1ToTarmac(),
                        ramseteCommands.getTarmacToBall2(),
                        new WaitCommand(3),
                        ramseteCommands.getBall2ToLoad2(),
                        new WaitCommand(1),
                        ramseteCommands.getLoad2ToShoot());
        Command intakeSequence = superstructure.deployAndRunIntake(() -> 13);

        Command shooterFeeder =
                CommandGroupBase.sequence(
                        superstructure
                                .runFeeder(() -> 8)
                                .alongWith(superstructure.accelerateWithMinMaxDistance(3.7, 3.8))
                                .withTimeout(
                                        Trajectories.path1Time
                                                + Trajectories.path2Time
                                                + Trajectories.path3Time * 0.9),
                        superstructure
                                .autoAccelerateAndShoot()
                                .withTimeout(3 + Trajectories.path3Time * 0.1),
                        new WaitCommand(Trajectories.path4Time * 0.5),
                        superstructure
                                .runFeeder(() -> 8)
                                .withTimeout(
                                        Trajectories.path4Time * 0.5
                                                + Trajectories.path5Time * 0.5),
                        superstructure
                                .runFeeder(() -> 8)
                                .alongWith(superstructure.accelerateWithMinMaxDistance(4.05, 4.15))
                                .withTimeout(Trajectories.path5Time * 0.4),
                        superstructure.autoAccelerateAndShoot());

        return CommandGroupBase.parallel(
                superstructure.disableCompressor(),
                drivetrainSequence,
                intakeSequence,
                shooterFeeder,
                turretSequence);
    }

    private final Command runFeederAndAccelerate() {
        return CommandGroupBase.parallel(
                superstructure.runFeeder(() -> 8),
                superstructure.flywheelCommands.variableVelocity(
                        superstructure::getAimedFlywheelSurfaceVel));
    }

    private final Command runFeederAndAccelerate(double minDistance, double maxDistance) {
        return CommandGroupBase.parallel(
                superstructure.runFeeder(() -> 8),
                superstructure.accelerateWithMinMaxDistance(minDistance, maxDistance));
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
                                        runFeederAndAccelerate()
                                                .withTimeout(Trajectories.path1Time * 0.95),
                                        superstructure
                                                .autoAccelerateAndShoot(1.2)
                                                .withTimeout(
                                                        Trajectories.path1Time * 0.05
                                                                + Trajectories.path2Time * 0.05
                                                                + 1.5)),
                                CommandGroupBase.sequence(
                                        runFeederAndAccelerate()
                                                .withTimeout(Trajectories.path2Time * 0.99),
                                        superstructure
                                                .autoAccelerateAndShoot(1.2)
                                                .withTimeout(
                                                        Trajectories.path2Time * 0.05
                                                                + Trajectories.path3Time * 0.05
                                                                + 1.2)),
                                CommandGroupBase.sequence(
                                        runFeederAndAccelerate()
                                                .withTimeout(Trajectories.path3Time * 0.95),
                                        superstructure
                                                .autoAccelerateAndShoot(4)
                                                .withTimeout(Trajectories.path4Time * 0.05 + 1)),
                                CommandGroupBase.sequence(
                                        runFeederAndAccelerate()
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
