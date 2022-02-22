package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import java.util.function.DoubleSupplier;
import team3647.frc2022.subsystems.ColumnTop;

public class ColumnTopCommands {
    private final ColumnTop columnTop;

    public ColumnTopCommands(ColumnTop columnTop) {
        this.columnTop = columnTop;
    }

    public Command getGoVariableVelocity(DoubleSupplier surfaceVelMpSFunction) {
        return new FunctionalCommand(
                () -> {},
                () -> columnTop.setSurfaceVelocity(surfaceVelMpSFunction.getAsDouble()),
                interrupted -> columnTop.setOpenloop(0),
                () -> false,
                columnTop);
    }
}
