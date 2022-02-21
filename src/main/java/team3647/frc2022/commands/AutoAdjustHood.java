// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.*;
import java.util.function.Supplier;
import team3647.frc2022.subsystems.Hood;
import team3647.lib.vision.AimingParameters;

public class AutoAdjustHood extends CommandBase {
    private final Hood hood;
    private final Supplier<AimingParameters> aimParameters;
    private final Function<Double, Double> getHoodAngleFromDistance;
    /** Creates a new AutoAdjustHood. */
    public AutoAdjustHood(
            Hood hood,
            Supplier<AimingParameters> aimParameters,
            Function<Double, Double> getHoodAngleFromDistance) {
        this.hood = hood;
        this.aimParameters = aimParameters;
        this.getHoodAngleFromDistance = getHoodAngleFromDistance;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        var aimingParam = aimParameters.get();
        double hoodAngle = getHoodAngleFromDistance.apply(aimingParam.getRangeMeters());

        hood.setAngleMotionMagic(hoodAngle);
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
