// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import team3647.frc2022.constants.LEDConstants;

public class StatusLED extends SubsystemBase {
    public static class Color {
        public static final Color WHITE = new Color(255, 255, 255);
        public static final Color BLACK = new Color(0, 0, 0);
        public static final Color RED = new Color(255, 0, 0);
        public static final Color ORANGE = new Color(255, 100, 0);
        public static final Color YELLOW = new Color(255, 255, 0);
        public static final Color GREEN = new Color(0, 255, 0);
        public static final Color LIGHTBLUE = new Color(0, 255, 255);
        public static final Color BLUE = new Color(0, 255, 0);
        public static final Color PURPLE = new Color(150, 0, 255);
        public static final Color PINK = new Color(255, 0, 255);

        public final int red;
        public final int green;
        public final int blue;

        public Color(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }
    }

    private final CANdle candle;

    private double BlinkSpeed;

    /** Creates a new StatusLED. */
    public StatusLED() {
        this.candle = new CANdle(LEDConstants.CANdleID, "Status Light");
    }

    private Animation m_toAnimate = null;

    public void setColor(Color color, double blinkSpeed) {
        this.candle.setLEDs(color.red, color.green, color.blue);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
