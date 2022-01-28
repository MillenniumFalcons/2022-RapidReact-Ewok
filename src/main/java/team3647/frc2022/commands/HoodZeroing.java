// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2022.subsystems.Hood;

public class HoodZeroing extends CommandBase {
    private final Hood hood;
    private MedianFilter filter;
    private double medianVelocity;
    private boolean reachHighVelocity = false;
    private boolean haveReachedHighVelocity = false;
    private boolean reachHardStop = false;
    private final double lowVelocityThreshold;
    private final double highVelocityThreshold;
    private final double output;

    /** Creates a new HoodZeroing. */
    public HoodZeroing(
            Hood hood,
            double output,
            int sampleSize,
            double lowVelocityThreshold,
            double highVelocityThreshold) {
        this.hood = hood;
        this.output = output;
        this.highVelocityThreshold = highVelocityThreshold;
        this.lowVelocityThreshold = lowVelocityThreshold;
        filter = new MedianFilter(sampleSize);
        addRequirements(hood);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        this.hood.setOpenloop(output);
        medianVelocity = filter.calculate(hood.getVelocity());
        reachHighVelocity = (medianVelocity >= highVelocityThreshold);
        if (reachHighVelocity) {
            haveReachedHighVelocity = true;
        }
        reachHardStop =
                (medianVelocity <= lowVelocityThreshold && medianVelocity >= -lowVelocityThreshold);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        this.hood.end();
        this.hood.resetEncoder();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (reachHardStop && haveReachedHighVelocity) {
            return true;
        }
        return false;
    }
}
