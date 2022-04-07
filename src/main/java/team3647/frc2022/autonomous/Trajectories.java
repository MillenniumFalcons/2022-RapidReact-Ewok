package team3647.frc2022.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import java.util.List;
import team3647.frc2022.constants.DrivetrainConstants;

// previous Trajectories
public class Trajectories {
    private static final DifferentialDriveVoltageConstraint autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                    DrivetrainConstants.kFeedforward,
                    DrivetrainConstants.kDriveKinematics,
                    DrivetrainConstants.kNominalVoltage);
    private static final CentripetalAccelerationConstraint slowTurns =
            new CentripetalAccelerationConstraint(2);

    private static final TrajectoryConfig forwardTrajectoryConfigSlow =
            new TrajectoryConfig(
                            DrivetrainConstants.kMaxSpeedMetersPerSecondSlow,
                            DrivetrainConstants.kMaxAccelerationMetersPerSecondSquared / 2.0)
                    .setKinematics(DrivetrainConstants.kDriveKinematics)
                    .addConstraint(autoVoltageConstraint)
                    .setReversed(false);
    private static final TrajectoryConfig forwardTrajectoryConfig =
            new TrajectoryConfig(
                            DrivetrainConstants.kMaxSpeedMetersPerSecond,
                            DrivetrainConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(DrivetrainConstants.kDriveKinematics)
                    .addConstraint(autoVoltageConstraint)
                    .setReversed(false);

    private static final TrajectoryConfig reverseTrajectoryConfig =
            new TrajectoryConfig(
                            DrivetrainConstants.kMaxSpeedMetersPerSecond,
                            DrivetrainConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(DrivetrainConstants.kDriveKinematics)
                    .addConstraint(autoVoltageConstraint)
                    .setReversed(true);
    private static final TrajectoryConfig reverseTrajectorySlowTurnsConfig =
            new TrajectoryConfig(
                            DrivetrainConstants.kMaxSpeedMetersPerSecond,
                            DrivetrainConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(DrivetrainConstants.kDriveKinematics)
                    .addConstraints(List.of(autoVoltageConstraint, slowTurns))
                    .setReversed(true);

    private static final TrajectoryConfig reverseTrajectoryConfigSlow =
            new TrajectoryConfig(
                            DrivetrainConstants.kMaxSpeedMetersPerSecondSlow - 0.5,
                            DrivetrainConstants.kMaxAccelerationMetersPerSecondSquared / 2.0)
                    .setKinematics(DrivetrainConstants.kDriveKinematics)
                    .addConstraint(autoVoltageConstraint)
                    .setReversed(true);

    private static final Pose2d path1Start = AutoConstants.positionOnTarmacParallel;
    private static final Pose2d path1End = AutoConstants.bottomLeftBall1;
    private static final Pose2d path2Start = path1End;
    private static final Pose2d path2End = path1Start;
    private static final Pose2d path3Start = path2End;
    private static final Pose2d path3End = AutoConstants.bottomLeftBall2At20Left;
    private static final Pose2d path4Start = path3End;
    private static final Pose2d path4End = AutoConstants.bottomLeftBall3At32;
    private static final Pose2d path5Start = path4End;
    private static final Pose2d path5End = AutoConstants.shootPoint;
    // 5 ball
    public static Trajectory tarmacToBottomLeftBall1 =
            TrajectoryGenerator.generateTrajectory(
                    path1Start, List.of(), path1End, reverseTrajectoryConfig);
    public static Trajectory bottomLeftBall1ToTarmac =
            TrajectoryGenerator.generateTrajectory(
                    path2Start, List.of(), path2End, forwardTrajectoryConfig);
    public static Trajectory tarmacToBall2 =
            TrajectoryGenerator.generateTrajectory(
                    path3Start, List.of(), path3End, reverseTrajectorySlowTurnsConfig);
    public static Trajectory ball2ToLoad2 =
            TrajectoryGenerator.generateTrajectory(
                    path4Start, List.of(), path4End, reverseTrajectoryConfig);
    public static Trajectory load2ToShoot =
            TrajectoryGenerator.generateTrajectory(
                    path5Start, List.of(), path5End, forwardTrajectoryConfig);

    public static double path1Time = tarmacToBottomLeftBall1.getTotalTimeSeconds();
    public static double path2Time = bottomLeftBall1ToTarmac.getTotalTimeSeconds();
    public static double path3Time = tarmacToBall2.getTotalTimeSeconds();
    public static double path4Time = ball2ToLoad2.getTotalTimeSeconds();
    public static double path5Time = load2ToShoot.getTotalTimeSeconds();

    // 2 ball
    private static final Pose2d path6Start = AutoConstants.positionOnTarmacUpper;
    private static final Pose2d path6End = AutoConstants.upperBall1Straight;
    private static final Pose2d path7Start = path6End;
    private static final Pose2d path7End = AutoConstants.upperOtherColorBall1;
    private static final Pose2d path8Start = path7End;
    private static final Pose2d path8End = AutoConstants.transitionPointForUpperOtherBall2;
    private static final Pose2d path9Start = path8End;
    private static final Pose2d path9End = AutoConstants.upperOtherColorBall2;
    private static final Pose2d path10Start = path9End;
    private static final Pose2d path10End = AutoConstants.outtakeTwoOtherColor;

    public static Trajectory upperTarmacToUpperBall1 =
            TrajectoryGenerator.generateTrajectory(
                    path6Start, List.of(), path6End, reverseTrajectoryConfigSlow);
    public static Trajectory upperBall1ToOtherBall1 =
            TrajectoryGenerator.generateTrajectory(
                    path7Start, List.of(), path7End, reverseTrajectoryConfigSlow);
    public static Trajectory otherBall1ToTransition =
            TrajectoryGenerator.generateTrajectory(
                    path8Start, List.of(), path8End, forwardTrajectoryConfig);
    public static Trajectory transitionToOtherBall2 =
            TrajectoryGenerator.generateTrajectory(
                    path9Start, List.of(), path9End, reverseTrajectoryConfigSlow);
    public static Trajectory otherBall2ToHangar =
            TrajectoryGenerator.generateTrajectory(
                    path10Start,
                    List.of(
                            AutoConstants.transitionPointForDipper1,
                            AutoConstants.transitionPointForDipper2),
                    path10End,
                    forwardTrajectoryConfigSlow);

    public static double path6Time = upperTarmacToUpperBall1.getTotalTimeSeconds();
    public static double path7Time = upperBall1ToOtherBall1.getTotalTimeSeconds();
    public static double path8Time = otherBall1ToTransition.getTotalTimeSeconds();
    public static double path9Time = transitionToOtherBall2.getTotalTimeSeconds();
    public static double path10Time = otherBall2ToHangar.getTotalTimeSeconds();

    private static final Pose2d path11Start = AutoConstants.positionOnTarmacUpper;
    private static final Pose2d path11End = AutoConstants.upperBall1Curved;
    private static final Pose2d path12Start = path11End;
    private static final Pose2d path12End = AutoConstants.endingFlatPoint;

    public static Trajectory upperTarmacToCurvedBall1 =
            TrajectoryGenerator.generateTrajectory(
                    path11Start, List.of(), path11End, reverseTrajectoryConfig);
    public static Trajectory curvedBall1ToFlatOtherBall =
            TrajectoryGenerator.generateTrajectory(
                    path12Start,
                    List.of(
                            AutoConstants.otherColorBall1,
                            AutoConstants.loopPoint,
                            AutoConstants.flatLoopPoint),
                    path12End,
                    reverseTrajectoryConfig);

    public static double path11Time = upperTarmacToCurvedBall1.getTotalTimeSeconds();
    public static double path12Time = curvedBall1ToFlatOtherBall.getTotalTimeSeconds();
}
