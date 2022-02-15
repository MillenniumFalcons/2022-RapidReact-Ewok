// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.DoubleSupplier;
import team3647.frc2022.subsystems.ColumnBottom;
import team3647.frc2022.subsystems.ColumnTop;
import team3647.frc2022.subsystems.Flywheel;

public class ShootBall extends CommandBase {
    private final Flywheel flywheel;
    private final ColumnTop columnTop;
    private final ColumnBottom columnBottom;
    private final DoubleSupplier surfaceVel;

    public ShootBall(
            Flywheel flywheel,
            ColumnTop columnTop,
            ColumnBottom columnBottom,
            DoubleSupplier surfaceVel) {
        this.flywheel = flywheel;
        this.columnTop = columnTop;
        this.columnBottom = columnBottom;
        this.surfaceVel = surfaceVel;
        addRequirements(flywheel, columnTop, columnBottom);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double speed = surfaceVel.getAsDouble();
        if (speed < 1) {
            columnTop.setOpenloop(0);
            columnBottom.setOpenloop(0);
            flywheel.setOpenloop(0);
        } else {
            flywheel.setSurfaceSpeed(speed);
            columnTop.setOpenloop(.6);
            if (flywheel.getVelocity() > speed - 0.1) {
                columnBottom.setOpenloop(0.4);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        flywheel.setOpenloop(0);
        columnTop.setOpenloop(0);
        columnBottom.setOpenloop(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
