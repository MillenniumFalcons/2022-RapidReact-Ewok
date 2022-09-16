package team3647.frc2022.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

// previous contants.cField
public class AutoConstants {
    // acceleration values
    public static final double kMaxSpeedMetersPerSecond = 4;
    public static final double kMaxAccelerationMetersPerSecondSquared = 4;
    public static final double slowBoiAutoAccel = 1.5;
    public static final double regBoiAutoAccel = 3.5;
    public static final double fastBoiAutoAccel = 4.5;

    // 2022 5 ball
    public static final Pose2d positionOnTarmacParallel =
            new Pose2d(7.62, 1.8, Rotation2d.fromDegrees(90));
    public static final Pose2d bottomMidPoint = new Pose2d(7.62, 0.65, Rotation2d.fromDegrees(90));
    public static final Pose2d bottomLeftBall1 = new Pose2d(7.62, 1, Rotation2d.fromDegrees(90));
    public static final Pose2d positionEndPath2 =
            new Pose2d(7.62, 2.74, Rotation2d.fromDegrees(90));
    public static final Pose2d bottomLeftBall2At20Left =
            new Pose2d(5.17, 1.92, Rotation2d.fromDegrees(-11.22));
    public static final Pose2d shootPoint = new Pose2d(5.3, 2, Rotation2d.fromDegrees(-20));
    public static final Pose2d farShootPoint = new Pose2d(2.8, 2.4, Rotation2d.fromDegrees(40));
    public static final Pose2d bottomLeftBall3At32 =
            new Pose2d(1.3, 1.48, Rotation2d.fromDegrees(40));
    public static final Pose2d positionEndPath5 =
            new Pose2d(3.74, 3.32, Rotation2d.fromDegrees(40));

    public static final Pose2d positionOnTarmacUpper =
            new Pose2d(6.16, 5.13, Rotation2d.fromDegrees(-44.45));
    public static final Pose2d almostUpperBall1Straight =
            new Pose2d(5.63, 5.65, Rotation2d.fromDegrees(-44.45));
    public static final Pose2d upperBall1Straight =
            new Pose2d(5, 6.17, Rotation2d.fromDegrees(-44.45));
    public static final Pose2d highShoot = new Pose2d(5.79, 5.49, Rotation2d.fromDegrees(-44.45));
    public static final Pose2d upperOtherColorBall1 =
            new Pose2d(5.68, 7.26, Rotation2d.fromDegrees(180));

    public static final Translation2d transitionPointForS = new Translation2d(4.55, 5.31);
    public static final Pose2d transitionPointForUpperOtherBall2 =
            new Pose2d(2.92, 3.28, Rotation2d.fromDegrees(180));
    public static final Pose2d upperOtherColorBall2 =
            new Pose2d(4.54, 3.28, Rotation2d.fromDegrees(180));
    public static final Translation2d transitionPointForDipper1 = new Translation2d(1.87, 4.58);
    public static final Translation2d transitionPointForDipper2 = new Translation2d(1.68, 6.27);
    public static final Pose2d outtakeTwoOtherColor =
            new Pose2d(5.64, 6.8, Rotation2d.fromDegrees(0));

    public static final Pose2d positionOnTarmacHighPoint =
            new Pose2d(6.7, 5.75, Rotation2d.fromDegrees(-44.45));
    public static final Pose2d upperBall1Curved =
            new Pose2d(4.95, 5.91, Rotation2d.fromDegrees(46));
    public static final Translation2d otherColorBall1 = new Translation2d(4.2, 3.15);
    public static final Translation2d loopPoint = new Translation2d(1.8, 5.46);
    public static final Translation2d flatLoopPoint = new Translation2d(3.21, 6.94);
    public static final Pose2d endingFlatPoint =
            new Pose2d(5.8, 7.07, Rotation2d.fromDegrees(-180));
}
