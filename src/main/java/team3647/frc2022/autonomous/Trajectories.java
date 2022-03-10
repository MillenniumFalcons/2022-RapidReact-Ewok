package team3647.frc2022.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
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
    private static final Pose2d path5End = AutoConstants.bottomLeftBall2At20Left;
    // 5 ball
    public static Trajectory tarmacToBottomLeftBall1 =
            TrajectoryGenerator.generateTrajectory(
                    path1Start, List.of(), path1End, reverseTrajectoryConfig);
    public static Trajectory bottomLeftBall1ToTarmac =
            TrajectoryGenerator.generateTrajectory(
                    path2Start, List.of(), path2End, forwardTrajectoryConfig);
    public static Trajectory tarmacToBall2 =
            TrajectoryGenerator.generateTrajectory(
                    path3Start, List.of(), path3End, reverseTrajectoryConfigSlow);
    public static Trajectory ball2ToLoad2 =
            TrajectoryGenerator.generateTrajectory(
                    path4Start, List.of(), path4End, reverseTrajectoryConfig);
    public static Trajectory load2ToShoot =
            TrajectoryGenerator.generateTrajectory(
                    path5Start, List.of(), path5End, forwardTrajectoryConfig);

    // 2 ball
    public static Trajectory upperTarmacToUpperBall1 =
            TrajectoryGenerator.generateTrajectory(
                    AutoConstants.upperPositionOnTarmac,
                    List.of(),
                    AutoConstants.upperBall1,
                    reverseTrajectoryConfig);
    public static Trajectory upperBall1ToUpperTarmac =
            TrajectoryGenerator.generateTrajectory(
                    AutoConstants.upperBall1,
                    List.of(),
                    AutoConstants.upperPositionOnTarmac,
                    forwardTrajectoryConfigSlow);

    // straight to test characterization
    public static Trajectory straightPath =
            TrajectoryGenerator.generateTrajectory(
                    AutoConstants.startingStraight,
                    List.of(),
                    AutoConstants.endingTurn,
                    forwardTrajectoryConfigSlow);
}
