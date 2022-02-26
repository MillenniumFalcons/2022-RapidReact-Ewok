// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.subsystems;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class StatusLED extends SubsystemBase {
    public static class Color {}

    private final CANdle candle;

    /** Creates a new StatusLED. */
    public StatusLED(CANdle candle) {
        this.candle = candle;
    }

    public void setColor(Color color, double speed) {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
