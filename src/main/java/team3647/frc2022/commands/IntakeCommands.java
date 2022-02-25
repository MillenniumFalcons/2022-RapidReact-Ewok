package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.function.DoubleSupplier;
import team3647.frc2022.subsystems.Intake;

public class IntakeCommands {
    private final Intake intake;

    public IntakeCommands(Intake intake) {
        this.intake = intake;
    }

    public Command getDeployIntake() {
        return new InstantCommand(intake::extend);
    }

    public Command getRetractIntake() {
        return new InstantCommand(intake::retract);
    }

    public Command getRunIntakeOpenloop(DoubleSupplier percentOut) {
        return new FunctionalCommand(
                () -> {},
                () -> intake.setOpenloop(percentOut.getAsDouble()),
                interrupted -> {
                    intake.setOpenloop(0);
                    intake.retract();
                },
                () -> false,
                intake);
    }

    public Command getRunIntakeClosedloop(DoubleSupplier surfaceVel) {
        return new FunctionalCommand(
                () -> {},
                () -> intake.setSurfaceVelocity(surfaceVel.getAsDouble()),
                interrupted -> {
                    intake.setOpenloop(0);
                    intake.retract();
                },
                () -> false,
                intake);
    }

    public Command getIntakeSequnce(Command intakingCommand) {
        return getDeployIntake().andThen(intakingCommand);
    }

    public Command getEndSequence() {
        return getRetractIntake().andThen(new RunCommand(intake::end, intake));
    }
}
