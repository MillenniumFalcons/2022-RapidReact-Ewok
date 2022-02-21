// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import java.util.function.*;
import java.util.function.Supplier;
import team3647.frc2022.subsystems.Ballstopper;
import team3647.frc2022.subsystems.ColumnBottom;
import team3647.frc2022.subsystems.ColumnTop;
import team3647.frc2022.subsystems.Flywheel;
import team3647.lib.vision.AimingParameters;

public class AutoShoot extends ParallelCommandGroup {
    private final Flywheel flywheel;
    private final ColumnTop columnTop;
    private final ColumnBottom columnBottom;
    private final Ballstopper stopper;
    private final Supplier<AimingParameters> aimParameters;
    private final Function<Double, Double> getShooterSpeedFromDistance;
    /** Creates a new AutoShoot. */
    public AutoShoot(
            Flywheel flywheel,
            ColumnTop columnTop,
            ColumnBottom columnBottom,
            Ballstopper stopper,
            Supplier<AimingParameters> aimParameters,
            Function<Double, Double> getShooterSpeedFromDistance) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.flywheel = flywheel;
        this.columnTop = columnTop;
        this.columnBottom = columnBottom;
        this.aimParameters = aimParameters;
        this.stopper = stopper;
        this.getShooterSpeedFromDistance = getShooterSpeedFromDistance;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        stopper.extend();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var aimingParam = aimParameters.get();
        double speed = getShooterSpeedFromDistance.apply(aimingParam.getRangeMeters());
        if (speed < 1) {
            columnTop.setOpenloop(0);
            columnBottom.setOpenloop(0);
            flywheel.setOpenloop(0);
        } else {
            flywheel.setSurfaceSpeed(speed - 0.5);
            columnTop.setSurfaceVelocity(speed);
            if (flywheel.getVelocity() > speed - 0.5 - 0.1) {
                columnBottom.setOpenloop(0.4);
                stopper.retract();
            } else {
                stopper.extend();
                columnBottom.setOpenloop(0);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        columnBottom.setOpenloop(0);
        columnTop.setOpenloop(0);
        flywheel.setOpenloop(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
