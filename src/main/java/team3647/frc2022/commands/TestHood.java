package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import team3647.frc2022.subsystems.Hood;

public class TestHood extends CommandBase {
    private final Hood hood;
    private final DoubleSupplier positionDeg;

    public TestHood(Hood hood, DoubleSupplier positiondeg) {
        this.hood = hood;
        this.positionDeg = positiondeg;
        addRequirements(hood);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        hood.setAngleMotionMagic(positionDeg.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        hood.end();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
