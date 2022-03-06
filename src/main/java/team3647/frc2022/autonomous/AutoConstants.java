package team3647.frc2022.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
            new Pose2d(
                    Units.inchesToMeters(296.541),
                    Units.inchesToMeters(66.578),
                    Rotation2d.fromDegrees(90));
    public static final Pose2d bottomLeftBall1 =
            new Pose2d(
                    Units.inchesToMeters(296.541),
                    Units.inchesToMeters(18.21),
                    Rotation2d.fromDegrees(90));
    public static final Translation2d bottomLeftBall2 =
            new Translation2d(Units.inchesToMeters(197.593), Units.inchesToMeters(73.11));
    public static final Pose2d bottomLeftBall3 =
            new Pose2d(
                    Units.inchesToMeters(40),
                    Units.inchesToMeters(45.684),
                    Rotation2d.fromDegrees(45));

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

    public static final Pose2d startingStraight =
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), Rotation2d.fromDegrees(0));
    public static final Pose2d endingTurn =
            new Pose2d(
                    Units.inchesToMeters(60),
                    Units.inchesToMeters(-60),
                    Rotation2d.fromDegrees(-90));
}
