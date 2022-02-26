// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3647.frc2022.constants.LEDConstants;

public class StatusLED extends SubsystemBase {
    // public static class LEDColor {}

    private final CANdle candle;

    private double BlinkSpeed;

    /** Creates a new StatusLED. */
    public StatusLED() {
        this.candle = new CANdle(LEDConstants.CANdleID, "Status Light");
    }

    private Animation m_toAnimate = null;

    public void setColor(String color, double BlinkSpeed) {
        this.BlinkSpeed = BlinkSpeed;
        if (color == "red") {
            m_toAnimate = new StrobeAnimation(255, 0, 0, 0, BlinkSpeed, LEDConstants.numofLED);
            candle.animate(m_toAnimate);
        } else if (color == "orange") {
            m_toAnimate = new StrobeAnimation(255, 165, 0, 0, BlinkSpeed, LEDConstants.numofLED);
            candle.animate(m_toAnimate);
        } else if (color == "yellow") {
            m_toAnimate = new StrobeAnimation(255, 255, 0, 0, BlinkSpeed, LEDConstants.numofLED);
            candle.animate(m_toAnimate);
        } else if (color == "green") {
            m_toAnimate = new StrobeAnimation(0, 255, 0, 0, BlinkSpeed, LEDConstants.numofLED);
            candle.animate(m_toAnimate);
        } else if (color == "blue") {
            m_toAnimate = new StrobeAnimation(0, 0, 255, 0, BlinkSpeed, LEDConstants.numofLED);
            candle.animate(m_toAnimate);
        } else if (color == "purple") {
            m_toAnimate = new StrobeAnimation(255, 0, 255, 0, BlinkSpeed, LEDConstants.numofLED);
            candle.animate(m_toAnimate);
        } else if (color == "white") {
            m_toAnimate = new StrobeAnimation(255, 0, 255, 255, BlinkSpeed, LEDConstants.numofLED);
            candle.animate(m_toAnimate);
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
