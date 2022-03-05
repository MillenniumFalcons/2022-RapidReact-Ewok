package team3647.frc2022.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import java.util.function.DoubleSupplier;
import team3647.frc2022.constants.ClimberConstants;
import team3647.frc2022.subsystems.PivotClimber;

public class ClimberCommands {
    private final PivotClimber climber;

    public ClimberCommands(PivotClimber climber) {
        this.climber = climber;
    }

    public Command climberToLength(double lengthMeters) {
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

    public class ClimberEightInchesUp extends CommandBase {
        double lengthMeters = ClimberConstants.kLengthFromStaticHooksToAboveBar;

        @Override
        public void initialize() {
            lengthMeters =
                    (climber.getLeftPosition() + climber.getRightPosition()) / 2
                            + Units.inchesToMeters(8);
        }

        @Override
        public void execute() {
            climber.moveMotionMagic(lengthMeters);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(climber.getLeftPosition() - lengthMeters) < 0.0254
                    && Math.abs(climber.getRightPosition() - lengthMeters) < 0.0254;
        }
    }

    public Command deploy() {
        return new InstantCommand(climber::setAngled)
                .andThen(new PrintCommand("Deploying Climber"))
                .andThen(new WaitCommand(0.1))
                .andThen(climberToLength(ClimberConstants.kLengthJustOverLowBar))
                .andThen(climber::setStraight);
    }

    public Command toNextRung() {
        return new ClimberEightInchesUp()
                .andThen(climber::setAngled)
                .andThen(new WaitCommand(0.1))
                .andThen(climberToLength(ClimberConstants.kMaxLengthAngled));
        // .andThen(climber::setStraight);
    }

    public Command setAngled() {
        return new InstantCommand(climber::setAngled);
    }

    public Command setStraight() {
        return new InstantCommand(climber::setStraight);
    }

    public Command openLoopControl(DoubleSupplier percentout) {
        return new FunctionalCommand(
                () -> {},
                () -> climber.setOpenloop(percentout.getAsDouble()),
                (interrupted) -> climber.setOpenloop(0),
                () -> false,
                climber);
    }
}
