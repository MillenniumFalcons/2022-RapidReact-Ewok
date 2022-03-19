package team3647.frc2022.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

// previous contants.cField
public class AutoConstants {
    // acceleration values
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;
    public static final double slowBoiAutoAccel = 1;
    public static final double regBoiAutoAccel = 3;
    public static final double fastBoiAutoAccel = 4;

    // 2022 5 ball
    public static final Pose2d positionOnTarmacParallel =
            new Pose2d(7.62, 1.8, Rotation2d.fromDegrees(90));
    public static final Pose2d bottomLeftBall1 = new Pose2d(7.62, 0.8, Rotation2d.fromDegrees(90));
    public static final Pose2d positionEndPath2 =
            new Pose2d(7.62, 2.74, Rotation2d.fromDegrees(90));
    public static final Pose2d bottomLeftBall2At20Left =
            new Pose2d(5.69, 1.73, Rotation2d.fromDegrees(-20));
    public static final Pose2d bottomLeftBall3At32 =
            new Pose2d(1.35, 1.35, Rotation2d.fromDegrees(40));
    public static final Pose2d positionEndPath5 =
            new Pose2d(3.74, 3.32, Rotation2d.fromDegrees(40));

    public static final Pose2d upperPositionOnTarmac =
            new Pose2d(
                    Units.inchesToMeters(250),
                    Units.inchesToMeters(200),
                    Rotation2d.fromDegrees(-38.65980825));
    public static final Pose2d upperBall1 =
            new Pose2d(
                    Units.inchesToMeters(192.5),
                    Units.inchesToMeters(242),
                    Rotation2d.fromDegrees(-38.65980825));
}
