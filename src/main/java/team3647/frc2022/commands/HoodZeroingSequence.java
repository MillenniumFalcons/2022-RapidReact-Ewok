// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import team3647.frc2022.subsystems.Hood;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HoodZeroingSequence extends SequentialCommandGroup {
    /** Creates a new HoodZeroingCommand. */
    public HoodZeroingSequence(
            Hood hood,
            double output,
            MedianFilter filter,
            double highVelocityThreshold,
            double lowVelocityThreshold,
            double physicalMinAngle) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new HoodZeroingHighVel(hood, output, highVelocityThreshold, filter),
                new HoodZeroingHardstop(hood, output, lowVelocityThreshold, filter));
    }
}
