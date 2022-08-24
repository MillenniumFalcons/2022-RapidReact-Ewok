package team3647.frc2022.autonomous;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;
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
