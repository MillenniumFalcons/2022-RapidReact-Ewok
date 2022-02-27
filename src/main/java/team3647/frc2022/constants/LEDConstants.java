package team3647.frc2022.constants;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.StrobeAnimation;

public class LEDConstants {
    public static int numofLED = 0;
    public static double speed = 0.0;
    public static int kCANdleId = 17;

    public static CANdle kCANdle = new CANdle(kCANdleId);

    public static class Color {
        public static final Color WHITE = new Color(255, 255, 255);
        public static final Color BLACK = new Color(0, 0, 0);
        public static final Color RED = new Color(255, 0, 0);
        public static final Color ORANGE = new Color(255, 100, 0);
        public static final Color YELLOW = new Color(235, 143, 52);
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

    public static Animation kBlinkWhite =
            new StrobeAnimation(Color.WHITE.red, Color.WHITE.green, Color.WHITE.blue, 255, 1, 511);
    public static Animation kBlinkYellow =
            new StrobeAnimation(Color.RED.red, Color.RED.green, Color.RED.blue, 128, 1, 511);
    public static CANdleConfiguration kCANdleConfig = new CANdleConfiguration();

    static {
        kCANdleConfig.vBatOutputMode = VBatOutputMode.Off;
        kCANdle.configAllSettings(kCANdleConfig);
    }
}
