package team3647.frc2022.autonomous;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import java.util.List;
import team3647.frc2022.constants.DrivetrainConstants;

// previous Trajectories
public class Trajectories {
    private static final DifferentialDriveVoltageConstraint autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(
                            DrivetrainConstants.kS, DrivetrainConstants.kV, DrivetrainConstants.kA),
                    DrivetrainConstants.kDriveKinematics,
                    DrivetrainConstants.kNominalVoltage);

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
                            DrivetrainConstants.kMaxSpeedMetersPerSecond,
                            AutoConstants.slowBoiAutoAccel)
                    .setKinematics(DrivetrainConstants.kDriveKinematics)
                    .addConstraint(autoVoltageConstraint)
                    .setReversed(true);
    private static final TrajectoryConfig forwardTrajectoryConfigFast =
            new TrajectoryConfig(
                            DrivetrainConstants.kMaxSpeedMetersPerSecond,
                            AutoConstants.fastBoiAutoAccel)
                    .setKinematics(DrivetrainConstants.kDriveKinematics)
                    .addConstraint(autoVoltageConstraint)
                    .setReversed(true);

    // 5 ball
    public static Trajectory tarmacToBottomLeftBall1 =
            TrajectoryGenerator.generateTrajectory(
                    AutoConstants.positionOnTarmacParallel,
                    List.of(),
                    AutoConstants.bottomLeftBall1,
                    reverseTrajectoryConfigSlow);
    public static Trajectory bottomLeftBall1ToTarmac =
            TrajectoryGenerator.generateTrajectory(
                    AutoConstants.bottomLeftBall1,
                    List.of(),
                    AutoConstants.positionOnTarmacParallel,
                    forwardTrajectoryConfig);
    public static Trajectory tarmacToBall2 =
            TrajectoryGenerator.generateTrajectory(
                    AutoConstants.positionOnTarmacParallel,
                    List.of(),
                    new Pose2d(
                            AutoConstants.bottomLeftBall2,
                            new Rotation2d(Units.degreesToRadians(-45))),
                    reverseTrajectoryConfig);
    public static Trajectory ball2ToLoad2 =
            TrajectoryGenerator.generateTrajectory(
                    new Pose2d(
                            AutoConstants.bottomLeftBall2,
                            new Rotation2d(Units.degreesToRadians(0))),
                    List.of(),
                    AutoConstants.bottomLeftBall3,
                    reverseTrajectoryConfig);
    public static Trajectory load2ToShoot =
            TrajectoryGenerator.generateTrajectory(
                    AutoConstants.bottomLeftBall3,
                    List.of(),
                    new Pose2d(
                            AutoConstants.bottomLeftBall2,
                            new Rotation2d(Units.degreesToRadians(45))),
                    forwardTrajectoryConfigFast);

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
                    forwardTrajectoryConfig);

    // straight to test characterization
    public static Trajectory straightPath =
            TrajectoryGenerator.generateTrajectory(
                    AutoConstants.startingStraight,
                    List.of(),
                    AutoConstants.endingStraight,
                    forwardTrajectoryConfig);
}
