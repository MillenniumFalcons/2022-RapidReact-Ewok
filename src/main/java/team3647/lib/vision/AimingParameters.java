package team3647.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

/** 254 AimingParameters class */
public class AimingParameters {
    private final double rangeMeters;
    private final Pose2d fieldToRobot;
    private final Pose2d fieldToTarget;
    private final Transform2d robotToTarget;
    private final Rotation2d robotToTargetRotation;
    private final double lastSeenTimestamp;
    private final double stability;

    public AimingParameters(
            Pose2d fieldToRobot, Pose2d fieldToTarget, double lastSeenTimestamp, double stability) {
        this.fieldToRobot = fieldToRobot;
        this.fieldToTarget = fieldToTarget;
        this.robotToTarget = fieldToTarget.minus(fieldToRobot);
        this.rangeMeters = robotToTarget.getTranslation().getNorm();
        this.robotToTargetRotation =
                new Rotation2d(
                        robotToTarget.getTranslation().getX(),
                        robotToTarget.getTranslation().getY());
        this.lastSeenTimestamp = lastSeenTimestamp;
        this.stability = stability;
    }

    public double getStability() {
        return stability;
    }

    public double getLastSeenTimestamp() {
        return lastSeenTimestamp;
    }

    public Rotation2d getRobotToGoalRotation() {
        return robotToTargetRotation;
    }

    public Pose2d getFieldToGoal() {
        return fieldToTarget;
    }

    public Pose2d getFieldToRobot() {
        return fieldToRobot;
    }

    public double getRangeMeters() {
        return rangeMeters;
    }
}
