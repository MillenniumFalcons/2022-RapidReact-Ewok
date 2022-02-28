// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3647.frc2022.constants.LEDConstants.Color;

public class StatusLED extends SubsystemBase {

    private final CANdle candle;

    private int blinkSpeed = 1;
    private Color ledColor = null;
    private Animation animation = null;
    /** Creates a new StatusLED. */
    public StatusLED(CANdle candle) {
        this.candle = candle;
    }

    public void setColor(Color color, int blinkSpeed) {
        ledColor = color;
        this.blinkSpeed = blinkSpeed;
        animation = null;
        candle.setLEDs(ledColor.red, ledColor.green, ledColor.blue);
    }

    public void setAnimation(Animation animation) {
        this.animation = animation;
        this.ledColor = null;
        candle.animate(this.animation);
    }

    @Override
    public void periodic() {
        if (this.ledColor != null) {
        } else if (animation != null) {

        }
    }
}
