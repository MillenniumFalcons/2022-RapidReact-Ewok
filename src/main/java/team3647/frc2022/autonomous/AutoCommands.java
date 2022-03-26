package team3647.frc2022.autonomous;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import team3647.frc2022.constants.HoodContants;
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

    public Command lowFiveClean() {
        Command turretSequence =
                superstructure.turretCommands.motionMagic(0).andThen(superstructure.aimTurret());
        Command drivetrainSequence =
                CommandGroupBase.sequence(
                        ramseteCommands.getTarmacToBottomLeftBall1(),
                        ramseteCommands.getBottomLeftBall1ToTarmac(),
                        ramseteCommands.getTarmacToBall2(),
                        new WaitUntilCommand(superstructure::hasTarget),
                        new WaitCommand(3),
                        ramseteCommands.getBall2ToLoad2(),
                        new WaitCommand(1),
                        ramseteCommands.getLoad2ToShoot());
        Command intakeSequence =
                superstructure
                        .deployAndRunIntake(() -> 13)
                        .withTimeout(Trajectories.path1Time + Trajectories.path2Time)
                        .andThen(
                                new WaitCommand(Trajectories.path3Time + 0.5),
                                superstructure.deployAndRunIntake(() -> 13));

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
                                .autoAccelerateAndShoot(1.2, 0.4, 0)
                                .withTimeout(1.5 + Trajectories.path3Time * 0.1),
                        new WaitUntilCommand(superstructure::hasTarget),
                        superstructure
                                .runFeeder(() -> 8)
                                .alongWith(superstructure.accelerateWithMinMaxDistance(3.1, 3.2))
                                .withTimeout(0.5),
                        superstructure.autoAccelerateAndShoot(1.2, 0.4, 0).withTimeout(1),
                        new WaitCommand(Trajectories.path4Time * 0.5),
                        superstructure
                                .runFeeder(() -> 8)
                                .withTimeout(
                                        Trajectories.path4Time * 0.5
                                                + Trajectories.path5Time * 0.5),
                        superstructure
                                .feederCommands
                                .runColumnBottomOut()
                                .alongWith(superstructure.columnTopCommands.getRunOutwards())
                                .withTimeout(0.1),
                        superstructure
                                .runFeeder(() -> 8)
                                .alongWith(superstructure.accelerateWithMinMaxDistance(3.1, 3.2))
                                .withTimeout(Trajectories.path5Time * 0.5 - 0.1),
                        new WaitCommand(0.1),
                        superstructure.autoAccelerateAndShoot(1.2, 0.4, 0));

        return CommandGroupBase.parallel(
                superstructure.disableCompressor(),
                drivetrainSequence,
                intakeSequence,
                shooterFeeder,
                turretSequence);
    }

    public Command getHighTwo() {
        Command drivetrainSequence =
                CommandGroupBase.sequence(
                        ramseteCommands.getTarmacToUpperBall1(),
                        new WaitCommand(2),
                        ramseteCommands.getUpperBall1ToOtherColorBall1(),
                        new WaitCommand(1));
        Command intakeSequence =
                superstructure
                        .deployAndRunIntake(() -> 13)
                        .withTimeout(Trajectories.path6Time + .2)
                        .andThen(new WaitCommand(1.8), superstructure.deployAndRunIntake(() -> 13));
        Command turretSequence =
                CommandGroupBase.sequence(
                        superstructure
                                .turretCommands
                                .motionMagic(0)
                                .andThen(superstructure.aimTurret())
                                .withTimeout(Trajectories.path6Time),
                        new WaitCommand(2),
                        superstructure.turretCommands.motionMagic(135));
        Command shooterFeederSequence =
                CommandGroupBase.sequence(
                        superstructure
                                .runFeeder(() -> 8)
                                .alongWith(superstructure.accelerateWithMinMaxDistance(3.04, 3.14))
                                .withTimeout(Trajectories.path6Time),
                        superstructure.autoAccelerateAndShoot(1.2, 0.4, 0).withTimeout(2),
                        superstructure
                                .runFeeder(() -> 8)
                                .alongWith(superstructure.accelerateWithMinMaxDistance(3.04, 3.14))
                                .withTimeout(Trajectories.path7Time),
                        superstructure.manualShootForAuto().withTimeout(2));

        return CommandGroupBase.parallel(
                superstructure.disableCompressor(),
                drivetrainSequence,
                intakeSequence,
                turretSequence,
                shooterFeederSequence);
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

    public Command lowGoalOnce() {
        return CommandGroupBase.parallel(
                superstructure.lowAccelerateAndShoot(),
                superstructure.turretCommands.motionMagic(0).perpetually(),
                superstructure.hoodCommands.motionMagic(HoodContants.kLowGoalAngle).perpetually());
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
}
