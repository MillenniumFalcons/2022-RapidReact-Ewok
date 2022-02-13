// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2022.subsystems.Turret;

public class TurretMotionMagic extends CommandBase {
    // private final Turret turret;
    /** Creates a new TurretMotionMagic. */
    public TurretMotionMagic(Turret turret, double value) {
        // this.turret = turret;
        // addRequirements(turret);
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
