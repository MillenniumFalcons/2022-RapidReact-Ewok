// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import team3647.frc2022.subsystems.PivotClimber;

public class ClimberLength extends CommandBase {
    private final PivotClimber climber;
    private final double lengthMeters;
    /** Creates a new ClimberPosition. */
    public ClimberLength(PivotClimber climber, double lengthMeters) {
        this.climber = climber;
        this.lengthMeters = lengthMeters;
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        climber.moveMotionMagic(lengthMeters);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        boolean leftAtPosition = Math.abs(climber.getLeftPosition() - lengthMeters) < 0.0254;
        boolean rightAtPosition = Math.abs(climber.getRightPosition() - lengthMeters) < 0.0254;
        return leftAtPosition && rightAtPosition;
    }
}
