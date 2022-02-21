package team3647.frc2022.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.function.DoubleSupplier;
import team3647.frc2022.constants.ClimberConstants;
import team3647.frc2022.subsystems.PivotClimber;

public class ClimberCommands {
    private final PivotClimber climber;

    public ClimberCommands(PivotClimber climber) {
        this.climber = climber;
    }

    public Command getClimberToLength(double lengthMeters) {
        return new FunctionalCommand(
                () -> {},
                // Execute
                () -> climber.moveMotionMagic(lengthMeters),
                interrupted -> {},
                // End condition
                () ->
                        Math.abs(climber.getLeftPosition() - lengthMeters) < 0.0254
                                && Math.abs(climber.getRightPosition() - lengthMeters) < 0.0254,
                climber);
    }

    public Command getClimberDeploy() {
        return new InstantCommand(climber::setAngled)
                .andThen(new WaitCommand(0.1))
                .andThen(getClimberToLength(ClimberConstants.kLengthJustOverLowBar))
                .andThen(climber::setStraight);
    }

    public Command getClimberToNextRung() {
        return getClimberToLength(ClimberConstants.kLengthFromStaticHooksToAboveBar)
                .andThen(climber::setAngled)
                .andThen(new WaitCommand(0.1))
                .andThen(getClimberToLength(ClimberConstants.kMaxLengthAngled))
                .andThen(climber::setAngled);
    }

    public Command getClimberOpenloop(DoubleSupplier upFunction, DoubleSupplier downFunction) {
        return new RunCommand(
                () -> climber.setOpenloop(upFunction.getAsDouble() - downFunction.getAsDouble()),
                climber);
    }
}
