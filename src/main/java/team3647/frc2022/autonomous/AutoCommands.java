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

    public Command sixBall() {
        // make sure 3 balls cannot be loaded at the same time
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
                        new WaitCommand(2.7),
                        ramseteCommands.getBottomLeftBall1ToTarmac(),
                        ramseteCommands.getTarmacToBall2(),
                        new WaitCommand(2),
                        ramseteCommands.getBall2ToLoad2(),
                        new WaitCommand(0.4),
                        ramseteCommands.getLoad2ToShoot());
        Command intakeSequence =
                CommandGroupBase.sequence(
                        new WaitCommand(Trajectories.path1Time + 2),
                        superstructure
                                .deployAndRunIntake(() -> 13)
                                .withTimeout(Trajectories.path2Time + Trajectories.path3Time + 0.2),
                        new WaitCommand(2),
                        superstructure.deployAndRunIntake(() -> 13));
        Command shooterFeeder =
                CommandGroupBase.sequence(
                        superstructure
                                .runFeeder(() -> 8)
                                .alongWith(superstructure.accelerateWithMinMaxDistance(3.23, 3.3))
                                .withTimeout(Trajectories.path1Time * 0.9),
                        superstructure
                                .autoAccelerateAndShoot()
                                .withTimeout(2.3 + Trajectories.path1Time * 0.1),
                        superstructure
                                .runFeeder(() -> 8)
                                .alongWith(superstructure.accelerateWithMinMaxDistance(3.1, 3.2))
                                .withTimeout(
                                        Trajectories.path2Time
                                                + Trajectories
                                                        .path3Time), // might not be enough to pick
                        // up the ball
                        superstructure.autoAccelerateAndShoot().withTimeout(2),
                        superstructure
                                .runFeeder(() -> 8)
                                .withTimeout(Trajectories.path4Time + Trajectories.path5Time * 0.5),
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

    public Command getHighTwoSendOnetoHangar() {
        Command drivetrainSequence =
                CommandGroupBase.sequence(
                        ramseteCommands.getTarmacToUpperBall1(),
                        new WaitCommand(2.5),
                        ramseteCommands.getUpperBall1ToOtherColorBall1());
        Command intakeSequence =
                CommandGroupBase.sequence(
                        superstructure
                                .deployAndRunIntake(() -> 13)
                                .withTimeout(Trajectories.path6Time + .1),
                        new WaitCommand(1.9),
                        superstructure
                                .deployAndRunIntake(() -> 13)
                                .withTimeout(Trajectories.path7Time + 0.5));
        Command turretSequence =
                superstructure
                        .turretCommands
                        .motionMagic(0)
                        .andThen(
                                new WaitCommand(Trajectories.path6Time * 0.5),
                                superstructure.aimTurret())
                        .withTimeout(Trajectories.path6Time + 3)
                        .andThen(superstructure.turretCommands.motionMagic(0));
        Command shooterFeederSequence =
                CommandGroupBase.sequence(
                        new WaitCommand(0.3),
                        superstructure
                                .autoAccelerateAndShoot()
                                .withTimeout(3 + Trajectories.path6Time),
                        superstructure.spitIntoHangar(7));

        return CommandGroupBase.parallel(
                superstructure.disableCompressor(),
                drivetrainSequence,
                intakeSequence,
                turretSequence,
                shooterFeederSequence);
    }

    //     public Command getHighTwoSendTwotoHangar() {
    //         Command drivetrainSequence =
    //                 CommandGroupBase.sequence(
    //                         ramseteCommands.getTarmacToUpperBall1(),
    //                         new WaitCommand(2),
    //                         ramseteCommands.getUpperBall1ToOtherColorBall1(),
    //                         ramseteCommands.getOtherColorBall1ToTransition(),
    //                         ramseteCommands.getTransitionToOtherColorBall2(),
    //                         ramseteCommands.getOtherColorBall2ToOuttake());
    //         Command intakeSequence =
    //                 CommandGroupBase.sequence(
    //                         superstructure
    //                                 .deployAndRunIntake(() -> 13)
    //                                 .withTimeout(Trajectories.path6Time + .1),
    //                         new WaitCommand(1.9),
    //                         superstructure
    //                                 .deployAndRunIntake(() -> 13)
    //                                 .withTimeout(Trajectories.path7Time),
    //                         new WaitCommand(Trajectories.path8Time),
    //                         superstructure
    //                                 .deployAndRunIntake(() -> 13)
    //                                 .withTimeout(Trajectories.path9Time));
    //         Command turretSequence =
    //
    // superstructure.turretCommands.motionMagic(0).andThen(superstructure.aimTurret());
    //         Command shooterFeederSequence =
    //                 superstructure
    //                         .autoAccelerateAndShoot(1.2, 0.4, 0)
    //                         .withTimeout(2.5 + Trajectories.path6Time)
    //                         .andThen(
    //                                 new WaitCommand(
    //                                         Trajectories.path7Time
    //                                                 + Trajectories.path8Time
    //                                                 + Trajectories.path9Time
    //                                                 + Trajectories.path10Time))
    //                         .andThen(outtake());

    //         return CommandGroupBase.parallel(
    //                 superstructure.disableCompressor(),
    //                 drivetrainSequence,
    //                 intakeSequence,
    //                 turretSequence,
    //                 shooterFeederSequence);
    //     }

    public final Command getHighTwoStay() {
        Command drivetrainSequence =
                CommandGroupBase.sequence(
                        ramseteCommands.getTarmacToUpperBall1(), new WaitCommand(2));
        Command intakeSequence =
                superstructure.deployAndRunIntake(() -> 13).withTimeout(Trajectories.path6Time);
        Command turretSequence =
                superstructure
                        .turretCommands
                        .motionMagic(0)
                        .andThen(
                                new WaitCommand(Trajectories.path6Time * 0.5),
                                superstructure.aimTurret())
                        .withTimeout(Trajectories.path6Time + 2);
        Command shooterFeederSequence =
                CommandGroupBase.sequence(
                        runFeederAndAccelerate(3.04, 3.14).withTimeout(Trajectories.path6Time),
                        superstructure.autoAccelerateAndShoot(1.2, 0.4, 0).withTimeout(2));

        return CommandGroupBase.parallel(
                superstructure.disableCompressor(),
                drivetrainSequence,
                intakeSequence,
                turretSequence,
                shooterFeederSequence);
    }

    public final Command getTwoGrabTwo() {
        Command drivetrainSequence =
                CommandGroupBase.sequence(
                        ramseteCommands.getUpperTarmactoCurvedBall1(),
                        new WaitCommand(3),
                        ramseteCommands.getCurvedBall1ToFlatPoint());
        Command intakeSequence =
                CommandGroupBase.sequence(
                        superstructure
                                .deployAndRunIntake(() -> 13)
                                .withTimeout(
                                        3 + Trajectories.path11Time + .3 * Trajectories.path12Time),
                        new WaitCommand(.5 * Trajectories.path12Time),
                        superstructure.deployAndRunIntake(() -> 13));
        Command turretSequence =
                CommandGroupBase.sequence(
                        superstructure.turretCommands.motionMagic(0),
                        superstructure.aimTurret().withTimeout(Trajectories.path11Time + 3),
                        superstructure.turretCommands.motionMagic(25));

        Command shooterFeederSequence =
                CommandGroupBase.sequence(
                        new WaitCommand(Trajectories.path11Time * 0.5),
                        superstructure
                                .autoAccelerateAndShoot()
                                .withTimeout(3 + Trajectories.path11Time * 0.5),
                        new WaitCommand(Trajectories.path12Time * 0.96),
                        superstructure.spitIntoHangar(7));
        return CommandGroupBase.parallel(
                drivetrainSequence,
                intakeSequence,
                turretSequence,
                shooterFeederSequence,
                superstructure.disableCompressor());
    }

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
