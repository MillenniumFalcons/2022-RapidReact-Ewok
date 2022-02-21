// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import team3647.frc2022.subsystems.ColumnBottom;
import team3647.frc2022.subsystems.Intake;
import team3647.frc2022.subsystems.VerticalRollers;

public class IntakeBallTest extends CommandBase {
    private final Intake intake;
    private final ColumnBottom columnBottom;
    private final VerticalRollers verticalRollers;
    private final DoubleSupplier output;

    /** Creates a new BallFeeding. */
    public IntakeBallTest(
            Intake intake,
            ColumnBottom columnBottom,
            VerticalRollers verticalRollers,
            DoubleSupplier output) {
        this.verticalRollers = verticalRollers;
        this.intake = intake;
        this.columnBottom = columnBottom;
        this.output = output;

        addRequirements(intake, columnBottom, verticalRollers);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        intake.extend();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double out = output.getAsDouble();
        intake.setOpenloop(-out * 0.4);
        columnBottom.setOpenloop(out * 0.3);
        verticalRollers.setOpenloop(0.4 * out);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        intake.retract();
        columnBottom.end();
        intake.end();
        verticalRollers.end();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
