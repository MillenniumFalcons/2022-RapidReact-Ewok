// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2022.subsystems.Hood;
import team3647.lib.wpi.Timer;

;

public class HoodZeroingHardstop extends CommandBase {
    private final Hood hood;
    private final Timer timer;
    private MedianFilter filter;
    private final double lowVelocityThreshold;
    private final double output;
    private final double time;
    private double medianVelocity;
    private boolean reachLowVelocity = false;
    private boolean reachHardStop = false;
    /** Creates a new HoodZeroingHardstop. */
    public HoodZeroingHardstop(
            Hood hood,
            double output,
            MedianFilter filter,
            double lowVelocityThreshold,
            double time) {
        this.hood = hood;
        this.output = output;
        this.filter = filter;
        this.lowVelocityThreshold = lowVelocityThreshold;
        this.time = time;
        timer = new Timer();
        addRequirements(hood);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        this.hood.setOpenloop(output);
        medianVelocity = filter.calculate(hood.getVelocity());
        reachLowVelocity =
                (medianVelocity <= lowVelocityThreshold && medianVelocity >= -lowVelocityThreshold);
        if (reachHardStop) {
            timer.start(); // no-op when timer is already running
            if (timer.get() > time) {
                reachHardStop = true;
            }

        } else {
            timer.reset();
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.hood.setOpenloop(0);
        this.hood.setEncoder(0); // set encoder to physically 0 angle
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return reachHardStop;
    }
}
