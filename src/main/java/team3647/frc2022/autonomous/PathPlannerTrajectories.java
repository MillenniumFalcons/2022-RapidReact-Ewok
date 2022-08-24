// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.autonomous;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class PathPlannerTrajectories {
    public static final PathPlannerTrajectory five1 =
            PathPlanner.loadPath(
                    "five-1",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    public static final Pose2d five1StartState = five1.getInitialPose();

    public static final PathPlannerTrajectory five2 =
            PathPlanner.loadPath(
                    "five-2",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    public static final Pose2d five2StartState = five2.getInitialPose();

    public static final PathPlannerTrajectory five3 =
            PathPlanner.loadPath(
                    "five-3",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    public static final Pose2d five3StartState = five3.getInitialPose();

    public static final PathPlannerTrajectory five4 =
            PathPlanner.loadPath(
                    "five-4",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    public static final Pose2d five4StartState = five4.getInitialPose();

    public static final PathPlannerTrajectory five5 =
            PathPlanner.loadPath(
                    "five-5",
                    AutoConstants.kMaxSpeedMetersPerSecond,
                    AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    public static final Pose2d five5StartState = five5.getInitialPose();
}
