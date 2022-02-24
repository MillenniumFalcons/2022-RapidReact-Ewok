package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.function.BooleanSupplier;
import team3647.frc2022.subsystems.ColumnBottom;
import team3647.frc2022.subsystems.ColumnTop;
import team3647.frc2022.subsystems.VerticalRollers;

public class FeederCommands {
    private final ColumnBottom columnBottom;
    private final ColumnTop columnTop;
    private final VerticalRollers verticalRollers;

    public FeederCommands(
            ColumnBottom columnBottom, ColumnTop columnTop, VerticalRollers verticalRollers) {
        this.columnBottom = columnBottom;
        this.columnTop = columnTop;
        this.verticalRollers = verticalRollers;
    }

    public Command getRunColmnBottomInwards() {
        return new FunctionalCommand(
                () -> {},
                () -> columnBottom.setOpenloop(0.3),
                interrupted -> columnBottom.setOpenloop(0),
                () -> false,
                columnBottom);
    }

    public Command getRunColmnBottomOutwards() {
        return new FunctionalCommand(
                () -> {},
                () -> columnBottom.setOpenloop(-0.3),
                interrupted -> columnBottom.setOpenloop(0),
                () -> false,
                columnBottom);
    }

    public Command getRunVerticalRollers() {
        return new FunctionalCommand(
                () -> {},
                () -> verticalRollers.setOpenloop(0.6),
                interrupted -> verticalRollers.setOpenloop(0),
                () -> false,
                verticalRollers);
    }

    public Command getFeedInwardsUntil(BooleanSupplier interruptOn) {
        return getRunColmnBottomInwards()
                .alongWith(getRunVerticalRollers())
                .withInterrupt(interruptOn);
    }

    public Command getFeedOutwardsUntil(BooleanSupplier interruptOn) {
        return getRunColmnBottomOutwards()
                .alongWith(getRunVerticalRollers())
                .withInterrupt(interruptOn);
    }

    public Command getFeedInwards() {
        return getRunColmnBottomInwards().alongWith(getRunVerticalRollers());
    }

    public Command getEndSequence() {
        return new RunCommand(columnTop::end, columnTop)
                .alongWith(new RunCommand(columnBottom::end, columnBottom))
                .alongWith(new RunCommand(verticalRollers::end, verticalRollers));
    }
}
