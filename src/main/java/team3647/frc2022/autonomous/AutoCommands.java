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

    public Command mikeJordanSixBall() {
        Command turretSequence =
                superstructure
                        .turretCommands
                        .motionMagic(0)
                        .andThen(
                                new WaitCommand(Trajectories.path1Time * 0.6),
                                superstructure.aimTurret());
        Command drivetrainSequence =
                CommandGroupBase.sequence(
                        ramseteCommands.getTarmacToBottomLeftBall1(),
                        new WaitCommand(3.1),
                        ramseteCommands.getBottomLeftBall1ToTarmac(),
                        // ramseteCommands.getTarmacToBall2(),
                        // ramseteCommands.getBall2ToLoad2()
                        ramseteCommands.getBall1ToLoad(),
                        ramseteCommands.getLoad2ToFarShoot());

        Command intakeSequence =
                CommandGroupBase.sequence(
                        new WaitCommand(Trajectories.path1Time + 1.2),
                        superstructure.deployAndRunIntake(() -> 10));

        Command shooterFeeder =
                CommandGroupBase.sequence(
                        superstructure
                                .runFeeder(() -> 8)
                                .alongWith(superstructure.accelerateWithMinMaxDistance(3.23, 3.3))
                                .withTimeout(Trajectories.path1Time + 0.1),
                        superstructure.autoAccelerateAndShoot().withTimeout(2.9),
                        superstructure
                                .runFeeder(() -> 8)
                                .alongWith(superstructure.accelerateWithMinMaxDistance(3.2, 3.25))
                                .withTimeout(
                                        Trajectories.path2Time
                                                + Trajectories.path3Time
                                                + Trajectories.path4Time
                                                + 0.2),
                        superstructure.autoAccelerateAndShoot());

        return CommandGroupBase.parallel(
                superstructure.disableCompressor(),
                drivetrainSequence,
                intakeSequence,
                shooterFeeder,
                turretSequence);
    }

    public Command lowFiveClean() {
        Command turretSequence =
                superstructure
                        .turretCommands
                        .motionMagic(0)
                        .andThen(
                                new WaitCommand(Trajectories.path1Time * 0.8),
                                superstructure.aimTurret());
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
                                .alongWith(superstructure.accelerateWithMinMaxDistance(3.23, 3.3))
                                .withTimeout(
                                        Trajectories.path1Time
                                                + Trajectories.path2Time
                                                + Trajectories.path3Time * 0.9),
                        superstructure
                                .autoAccelerateAndShoot()
                                .withTimeout(1.5 + Trajectories.path3Time * 0.1),
                        new WaitUntilCommand(superstructure::hasTarget),
                        superstructure
                                .runFeeder(() -> 8)
                                .alongWith(superstructure.accelerateWithMinMaxDistance(3.1, 3.2))
                                .withTimeout(0.5),
                        superstructure.autoAccelerateAndShoot().withTimeout(1),
                        new WaitCommand(Trajectories.path4Time * 0.5),
                        superstructure
                                .runFeeder(() -> 8)
                                .withTimeout(
                                        Trajectories.path4Time * 0.5
                                                + Trajectories.path5Time * 0.5),
                        superstructure
                                .runFeeder(() -> 8)
                                .alongWith(superstructure.accelerateWithMinMaxDistance(3.2, 3.25))
                                .withTimeout(Trajectories.path5Time * 0.5),
                        superstructure.autoAccelerateAndShoot());

        return CommandGroupBase.parallel(
                superstructure.disableCompressor(),
                drivetrainSequence,
                intakeSequence,
                shooterFeeder,
                turretSequence);
    }

    public Command getHighThreeSendOnetoHangar() {
        Command drivetrainSequence =
                CommandGroupBase.sequence(
                        ramseteCommands.getTarmacToShootHigh(),
                        new WaitCommand(2.2),
                        ramseteCommands.getTinyPathToLastHighBall(),
                        new WaitCommand(1.8),
                        ramseteCommands.getUpperBall1ToOtherColorBall1());
        Command intakeSequence =
                superstructure
                        .deployAndRunIntake(() -> 8)
                        .withTimeout(Trajectories.path6Time + 2.2 + 1.7)
                        .andThen(new WaitCommand(2))
                        .andThen(superstructure.deployAndRunIntake(() -> 10));
        Command turretSequence =
                superstructure
                        .turretCommands
                        .motionMagic(0)
                        .andThen(new WaitCommand(0.4))
                        .andThen(
                                superstructure
                                        .aimTurret()
                                        .withTimeout(
                                                Trajectories.path6Time
                                                        + 0.3
                                                        + 2.2
                                                        + Trajectories.path6TinyPathTime
                                                        + 1.8))
                        .andThen(superstructure.turretCommands.motionMagic(6));
        Command shooterFeederSequence =
                CommandGroupBase.sequence(
                        new WaitCommand(Trajectories.path6Time + 0.3),
                        superstructure.autoAccelerateAndShoot().withTimeout(2.2),
                        new WaitCommand(Trajectories.path6TinyPathTime),
                        superstructure.autoAccelerateAndShoot().withTimeout(1.8),
                        superstructure.spitIntoHangar(7));

        return CommandGroupBase.parallel(
                superstructure.disableCompressor(),
                drivetrainSequence,
                intakeSequence,
                turretSequence,
                shooterFeederSequence);

        // sus version
        // Command drivetrainSequence =
        //         CommandGroupBase.sequence(
        //                 ramseteCommands.getTarmacToShootHigh(),
        //                 new WaitCommand(2.2),
        //                 ramseteCommands.getTinyPathToLastHighBall(),
        //                 new WaitCommand(1.5),
        //                 ramseteCommands.getUpperBall1ToOtherColorBall1());
        // Command intakeSequence =
        //         new WaitCommand(Trajectories.path6Time + 2.4)
        //                 .andThen(superstructure.deployAndRunIntake(() -> 10).withTimeout(1.5))
        //                 .andThen(new WaitCommand(Trajectories.path7Time - 3))
        //                 .andThen(superstructure.deployAndRunIntake(() -> 10));
        // Command turretSequence =
        //         superstructure
        //                 .turretCommands
        //                 .motionMagic(0)
        //                 .andThen(new WaitCommand(0.4))
        //                 .andThen(
        //                         superstructure
        //                                 .aimTurret()
        //                                 .withTimeout(
        //                                         Trajectories.path6TinyPathTime
        //                                                 + 2.2
        //                                                 + Trajectories.path7Time
        //                                                 + 1))
        //                 .andThen(superstructure.turretCommands.motionMagic(0));
        // Command shooterFeederSequence =
        //         CommandGroupBase.sequence(
        //                 new WaitCommand(Trajectories.path6Time + 0.3),
        //                 superstructure.autoAccelerateAndShoot().withTimeout(2.2),
        //                 new WaitCommand(.5 * Trajectories.path6TinyPathTime),
        //                 superstructure
        //                         .autoAccelerateAndShoot()
        //                         .withTimeout(.5 * Trajectories.path6TinyPathTime + 1.5),
        //                 new WaitCommand(Trajectories.path7Time * .6 + 0.5),
        //                 superstructure.spitIntoHangar(7));

        // return CommandGroupBase.parallel(
        //         superstructure.disableCompressor(),
        //         drivetrainSequence,
        //         intakeSequence,
        //         turretSequence,
        //         shooterFeederSequence);
    }

    public final Command getHighThreeStay() {
        Command drivetrainSequence =
                CommandGroupBase.sequence(
                        ramseteCommands.getTarmacToShootHigh(),
                        new WaitCommand(2.2),
                        ramseteCommands.getTinyPathToLastHighBall(),
                        new WaitCommand(1.5));
        Command intakeSequence =
                new WaitCommand(Trajectories.path6Time + 2.4)
                        .andThen(superstructure.deployAndRunIntake(() -> 10).withTimeout(1.5));
        Command turretSequence =
                superstructure
                        .turretCommands
                        .motionMagic(0)
                        .andThen(new WaitCommand(0.4))
                        .andThen(superstructure.aimTurret());
        Command shooterFeederSequence =
                CommandGroupBase.sequence(
                        new WaitCommand(Trajectories.path6Time + 0.3),
                        superstructure.autoAccelerateAndShoot().withTimeout(2.2),
                        new WaitCommand(Trajectories.path6TinyPathTime),
                        superstructure.autoAccelerateAndShoot().withTimeout(1.5));

        return CommandGroupBase.parallel(
                superstructure.disableCompressor(),
                drivetrainSequence,
                intakeSequence,
                turretSequence,
                shooterFeederSequence);
    }

    public final Command getHighTwoNoMove() {
        Command drivetrainSequence =
                new WaitCommand(3.0)
                        .andThen(CommandGroupBase.sequence(ramseteCommands.getTarmacToShootHigh()));

        Command turretSequence =
                new WaitCommand(3.0)
                        .andThen(superstructure.turretCommands.motionMagic(0))
                        .andThen(new WaitCommand(0.4))
                        .andThen(superstructure.aimTurret());
        Command shooterFeederSequence =
                CommandGroupBase.sequence(
                        new WaitCommand(Trajectories.path6Time + 0.3 + 3.0),
                        superstructure.autoAccelerateAndShoot().withTimeout(2.2));

        return CommandGroupBase.parallel(
                superstructure.disableCompressor(),
                drivetrainSequence,
                turretSequence,
                shooterFeederSequence);
    }

    //     public final Command getTwoGrabTwo() {
    //         Command drivetrainSequence =
    //                 CommandGroupBase.sequence(
    //                         ramseteCommands.getUpperTarmactoCurvedBall1(),
    //                         new WaitCommand(3),
    //                         ramseteCommands.getCurvedBall1ToFlatPoint());
    //         Command intakeSequence =
    //                 CommandGroupBase.sequence(
    //                         superstructure
    //                                 .deployAndRunIntake(() -> 13)
    //                                 .withTimeout(
    //                                         3 + Trajectories.path11Time + .3 *
    // Trajectories.path12Time),
    //                         new WaitCommand(.5 * Trajectories.path12Time),
    //                         superstructure.deployAndRunIntake(() -> 13));
    //         Command turretSequence =
    //                 CommandGroupBase.sequence(
    //                         superstructure.turretCommands.motionMagic(0),
    //                         superstructure.aimTurret().withTimeout(Trajectories.path11Time + 3),
    //                         superstructure.turretCommands.motionMagic(25));

    //         Command shooterFeederSequence =
    //                 CommandGroupBase.sequence(
    //                         new WaitCommand(Trajectories.path11Time * 0.5),
    //                         superstructure
    //                                 .autoAccelerateAndShoot()
    //                                 .withTimeout(3 + Trajectories.path11Time * 0.5),
    //                         new WaitCommand(Trajectories.path12Time * 0.96),
    //                         superstructure.spitIntoHangar(7));
    //         return CommandGroupBase.parallel(
    //                 drivetrainSequence,
    //                 intakeSequence,
    //                 turretSequence,
    //                 shooterFeederSequence,
    //                 superstructure.disableCompressor());
    //     }

    private final Command outtake() {
        return CommandGroupBase.parallel(
                superstructure.feederCommands.runColumnBottomOut(),
                superstructure.columnTopCommands.getRunOutwards(),
                superstructure.flywheelCommands.openloop(-0.6));
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
}
