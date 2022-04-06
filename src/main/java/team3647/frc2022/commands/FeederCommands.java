package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import java.util.function.DoubleSupplier;
import team3647.frc2022.subsystems.ColumnBottom;
import team3647.frc2022.subsystems.ColumnTop;

public class FeederCommands {
    private final ColumnBottom columnBottom;
    private final ColumnTop columnTop;

    public FeederCommands(ColumnBottom columnBottom, ColumnTop columnTop) {
        this.columnBottom = columnBottom;
        this.columnTop = columnTop;
    }

    public Command runColumnBottom(DoubleSupplier surfaceSpeed) {
        return new FunctionalCommand(
                () -> {},
                () -> columnBottom.setSurfaceVelocity(surfaceSpeed.getAsDouble()),
                interrupted -> columnBottom.setOpenloop(0),
                () -> false,
                columnBottom);
    }

    public Command runColumnBottomOut() {
        return new FunctionalCommand(
                () -> {},
                () -> columnBottom.setOpenloop(-0.3),
                interrupted -> columnBottom.setOpenloop(0),
                () -> false,
                columnBottom);
    }

    public Command feedIn(DoubleSupplier columnSpeed) {
        return runColumnBottom(columnSpeed);
    }
}
