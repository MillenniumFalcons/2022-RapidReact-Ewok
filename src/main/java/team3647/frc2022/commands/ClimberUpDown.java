// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import team3647.frc2022.subsystems.PivotClimber;

public class ClimberUpDown extends CommandBase {
    private final PivotClimber pivotClimber;
    private final DoubleSupplier upFunction;
    private final DoubleSupplier downFunction;

    public ClimberUpDown(
            PivotClimber pivotClimber, DoubleSupplier upFunction, DoubleSupplier downFunction) {
        this.pivotClimber = pivotClimber;
        this.upFunction = upFunction;
        this.downFunction = downFunction;
        addRequirements(pivotClimber);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        pivotClimber.setOpenloop(upFunction.getAsDouble() - downFunction.getAsDouble());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
