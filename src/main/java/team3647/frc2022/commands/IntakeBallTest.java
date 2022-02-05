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
    private final VerticalRollers verticalRollers;
    private final ColumnBottom columnBottom;
    private final DoubleSupplier output;

    /** Creates a new BallFeeding. */
    public IntakeBallTest(
            Intake intake,
            VerticalRollers verticalRollers,
            ColumnBottom columnBottom,
            DoubleSupplier output) {
        this.intake = intake;
        this.verticalRollers = verticalRollers;
        this.columnBottom = columnBottom;
        this.output = output;

        addRequirements(intake, verticalRollers, columnBottom);
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
        intake.setOpenloop(-out * 0.5);
        verticalRollers.setOpenloop(out * 0.5);
        columnBottom.setOpenloop(out * 0.5);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      System.out.println("Stopping flywheel");
        intake.retract();
        verticalRollers.end();
        columnBottom.end();
        intake.end();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
