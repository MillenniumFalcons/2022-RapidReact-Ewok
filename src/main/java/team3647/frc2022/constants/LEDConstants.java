package team3647.frc2022.constants;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

public class LEDConstants {
    public static final int numofLED = 0;
    public static final double speed = 0.0;
    public static final int kCANdleId = 17;
    public static final double kBlinkSpeed = 1;

    public static CANdle kCANdle = new CANdle(kCANdleId);

    public static class Color {
        public static final Color WHITE = new Color(255, 255, 255);
        public static final Color BLACK = new Color(0, 0, 0);
        public static final Color RED = new Color(255, 0, 0);
        public static final Color ORANGE = new Color(255, 100, 0);
        public static final Color YELLOW = new Color(235, 255, 0);
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

    public static Animation kSolidWhite =
            new StrobeAnimation(Color.WHITE.red, Color.WHITE.green, Color.WHITE.blue, 255, 1, 512);
    public static Animation kBlinkWhiteFast =
            new SingleFadeAnimation(
                    Color.WHITE.red, Color.WHITE.green, Color.WHITE.blue, 255, kBlinkSpeed, 512);

    public static Animation kBlinkYellow =
            new SingleFadeAnimation(
                    Color.YELLOW.red, Color.YELLOW.green, Color.YELLOW.blue, 128, kBlinkSpeed, 512);
    public static Animation kSolidYellow =
            new StrobeAnimation(
                    Color.YELLOW.red, Color.YELLOW.green, Color.YELLOW.blue, 128, 1, 512);

    public static Animation kBlinkGreen =
            new StrobeAnimation(
                    Color.GREEN.red, Color.GREEN.green, Color.GREEN.blue, 128, kBlinkSpeed, 512);
    public static Animation kSolidkGreen =
            new StrobeAnimation(Color.GREEN.red, Color.GREEN.green, Color.GREEN.blue, 128, 1, 512);

    public static Animation kBlinkRed =
            new StrobeAnimation(
                    Color.RED.red, Color.RED.green, Color.RED.blue, 128, kBlinkSpeed, 512);

    public static Animation kSolidPink =
            new StrobeAnimation(
                    Color.PINK.red, Color.PINK.green, Color.PINK.blue, 128, kBlinkSpeed, 512);
    public static Animation kBlinkOrange =
            new StrobeAnimation(
                    Color.ORANGE.red, Color.ORANGE.green, Color.ORANGE.blue, 128, kBlinkSpeed, 512);
    public static Animation kSolidOrange =
            new StrobeAnimation(
                    Color.ORANGE.red, Color.ORANGE.green, Color.ORANGE.blue, 128, 1, 512);

    public static CANdleConfiguration kCANdleConfig = new CANdleConfiguration();

    static {
        kCANdleConfig.vBatOutputMode = VBatOutputMode.Off;
        kCANdleConfig.brightnessScalar = 0.7;
        kCANdle.configAllSettings(kCANdleConfig);
    }
}
