package team3647.frc2022.autonomous;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import team3647.frc2022.subsystems.Drivetrain;

public class AutoCommands {
    private final Drivetrain drivetrain;
    private final Command intakeFromGround;
    private final Command aimTurretHood;
    private final Command autoShoot;
    private final Command accelerateFlywheel;
    private final RamseteCommands ramseteCommands;

    public AutoCommands(
            Drivetrain drivetrain,
            DifferentialDriveKinematics driveKinematics,
            Command intakeFromGround,
            Command aimTurretHood,
            Command autoShoot,
            Command accelerateFlywheel) {
        this.drivetrain = drivetrain;
        this.intakeFromGround = intakeFromGround;
        this.aimTurretHood = aimTurretHood;
        this.autoShoot = autoShoot;
        this.accelerateFlywheel = accelerateFlywheel;
        ramseteCommands = new RamseteCommands(drivetrain, driveKinematics);
    }

    public Command getLowFive() {
        return ramseteCommands
                .tarmacToBottomLeftBall1
                .deadlineWith(intakeFromGround)
                .andThen(ramseteCommands.bottomLeftBall1ToTarmac.deadlineWith(accelerateFlywheel))
                .andThen(autoShoot)
                .andThen(
                        ramseteCommands.tarmacToBall2.deadlineWith(
                                intakeFromGround, accelerateFlywheel))
                .andThen(autoShoot)
                .andThen(ramseteCommands.ball2ToLoad2.deadlineWith(intakeFromGround))
                .andThen(ramseteCommands.load2ToShoot.deadlineWith(accelerateFlywheel))
                .andThen(autoShoot)
                .alongWith(aimTurretHood);
    }
}
