package team3647.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

/** 254 AimingParameters class */
public class AimingParameters {
    private final double rangeMeters;
    private final Pose2d fieldToRobot;
    private final Pose2d fieldToGoal;
    private final Transform2d robotToGoal;
    private final Rotation2d robotToGoalRotation;
    private final double lastSeenTimestamp;
    private final double stability;

    public AimingParameters(
            Pose2d fieldToRobot, Pose2d fieldToGoal, double lastSeenTimestamp, double stability) {
        this.fieldToRobot = fieldToRobot;
        this.fieldToGoal = fieldToGoal;
        this.robotToGoal = fieldToGoal.minus(fieldToRobot);
        this.rangeMeters = robotToGoal.getTranslation().getNorm();
        this.robotToGoalRotation =
                new Rotation2d(
                        robotToGoal.getTranslation().getX(), robotToGoal.getTranslation().getY());
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
        return robotToGoalRotation;
    }

    public Pose2d getFieldToGoal() {
        return fieldToGoal;
    }

    public Pose2d getFieldToRobot() {
        return fieldToRobot;
    }

    public double getRangeMeters() {
        return rangeMeters;
    }
}
