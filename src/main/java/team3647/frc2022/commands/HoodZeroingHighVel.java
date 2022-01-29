// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2022.subsystems.Hood;

public class HoodZeroingHighVel extends CommandBase {
    private final Hood hood;
    private final double output;
    private final double highVelocityThreshold;
    private MedianFilter filter;
    private double medianVelocity;
    private boolean reachHighVelocity = false;

    /** Creates a new HoodZeroing. */
    public HoodZeroingHighVel(
            Hood hood, double output, double highVelocityThreshold, MedianFilter filter) {
        this.hood = hood;
        this.output = output;
        this.filter = filter;
        this.highVelocityThreshold = highVelocityThreshold;
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
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return reachHighVelocity;
    }
}
