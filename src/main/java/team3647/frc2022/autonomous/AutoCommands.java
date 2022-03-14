package team3647.frc2022.autonomous;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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

    public Command getLowFive() {
        CommandGroupBase autoAndIntake =
                CommandGroupBase.sequence(
                        ramseteCommands
                                .getTarmacToBottomLeftBall1()
                                .deadlineWith(
                                        CommandGroupBase.parallel(
                                                superstructure.deployAndRunIntake(() -> 8),
                                                superstructure.flywheelCommands.variableVelocity(
                                                        superstructure
                                                                ::getAimedFlywheelSurfaceVel))),
                        superstructure.autoAccelerateAndShoot().withTimeout(2),
                        ramseteCommands.getBottomLeftBall1ToTarmac(),
                        ramseteCommands
                                .getTarmacToBall2()
                                .deadlineWith(
                                        CommandGroupBase.parallel(
                                                superstructure.deployAndRunIntake(() -> 8),
                                                superstructure.flywheelCommands.variableVelocity(
                                                        superstructure
                                                                ::getAimedFlywheelSurfaceVel))),
                        superstructure.autoAccelerateAndShoot().withTimeout(2),
                        ramseteCommands
                                .getBall2ToLoad2()
                                .deadlineWith(
                                        CommandGroupBase.parallel(
                                                superstructure.deployAndRunIntake(() -> 8),
                                                superstructure.flywheelCommands.variableVelocity(
                                                        superstructure
                                                                ::getAimedFlywheelSurfaceVel))),
                        superstructure.deployAndRunIntake(() -> 8).withTimeout(1.5),
                        ramseteCommands
                                .getLoad2ToShoot()
                                .deadlineWith(
                                        superstructure.deployAndRunIntake(() -> 8),
                                        superstructure.flywheelCommands.variableVelocity(
                                                superstructure::getAimedFlywheelSurfaceVel)),
                        superstructure.autoAccelerateAndShoot().withTimeout(5));
        return CommandGroupBase.parallel(
                superstructure.turretCommands.motionMagic(0).andThen(superstructure.aimTurret()),
                autoAndIntake);
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
