package team3647.frc2022.constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import team3647.lib.vision.IVisionCamera.CamConstants;
import team3647.lib.vision.VisionTargetConstants;

public class VisionConstants {
    public static String kLimelightIP = "10.36.47.16";
    public static double kNetworkLatency = 0.06; // seconds
    public static double kCameraHeightMeters = Units.inchesToMeters(40.5);
    public static double kTopOfTapeHeightMeters = Units.inchesToMeters(101);
    public static double kGoalDiameterMeters = Units.feetToMeters(5);
    public static double kBottomOfTapeHeightMeters = Units.inchesToMeters(99);
    public static Rotation2d kHorizontalToLens = Rotation2d.fromDegrees(45);
    public static double kVPH = 2.0 * Math.tan(Math.toRadians(49.7) / 2.0);
    public static double kVPW = 2.0 * Math.tan(Math.toRadians(59.8) / 2.0);
    public static CamConstants limelightConstants =
            new CamConstants(kCameraHeightMeters, kHorizontalToLens, kVPH, kVPW);
    public static VisionTargetConstants kCenterGoalTargetConstants =
            new VisionTargetConstants(
                    kTopOfTapeHeightMeters, kBottomOfTapeHeightMeters, 4, 2, kGoalDiameterMeters);
}
