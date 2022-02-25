// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
import team3647.frc2022.subsystems.ColumnBottom;

public class FeederSensoredBalls extends CommandBase {
    private final ColumnBottom columnBottom;
    private final BooleanSupplier topSensor;
    private final BooleanSupplier middleSensor;
    /** Creates a new IntakeSensoredBalls. */
    public FeederSensoredBalls(
            ColumnBottom columnBottom, BooleanSupplier topSensor, BooleanSupplier middleSensor) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.columnBottom = columnBottom;
        this.topSensor = topSensor;
        this.middleSensor = middleSensor;
        addRequirements(columnBottom);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double output = 0;
        if (topSensor.getAsBoolean() && middleSensor.getAsBoolean()) {
            output = 0.0;
        } else {
            output = 0.3;
        }

        System.out.println(output);

        columnBottom.setOpenloop(output);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        columnBottom.setOpenloop(0.0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
