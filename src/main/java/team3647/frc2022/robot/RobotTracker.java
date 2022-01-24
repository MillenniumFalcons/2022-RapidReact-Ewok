package team3647.frc2022.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import team3647.frc2022.subsystems.Drivetrain;
import team3647.frc2022.subsystems.Turret;

/** Inspired by (basically copied lol) 254 RobotState class */
public class RobotTracker {
    private final double kBufferLengthSeconds;

    private final TimeInterpolatableBuffer<Pose2d> fieldToRobot;
    private final TimeInterpolatableBuffer<Pose2d> robotToTurret;
    private final Translation2d kRobotToTurretFixed;
    private final Drivetrain drivetrain;
    private final Turret turret;
    private Twist2d measuredVelocity = new Twist2d();

    public RobotTracker(
            double bufferLengthSeconds,
            Drivetrain drivetrain,
            Turret turret,
            Translation2d robotToTurretFixed) {
        this.kBufferLengthSeconds = bufferLengthSeconds;
        this.drivetrain = drivetrain;
        this.turret = turret;
        this.kRobotToTurretFixed = robotToTurretFixed;
        fieldToRobot = TimeInterpolatableBuffer.createBuffer(kBufferLengthSeconds);
        robotToTurret = TimeInterpolatableBuffer.createBuffer(kBufferLengthSeconds);
    }

    public void update() {
        addFieldToRobotObservation(drivetrain.getTimestamp(), drivetrain.getPose());
        addTurretRotationObservation(
                turret.getTimestamp(),
                new Pose2d(kRobotToTurretFixed, Rotation2d.fromDegrees(turret.getPosition())));
    }

    public void addFieldToRobotObservation(double timestamp, Pose2d observation) {
        fieldToRobot.addSample(timestamp, observation);
    }

    public void addTurretRotationObservation(double timestamp, Pose2d observation) {
        robotToTurret.addSample(timestamp, observation);
    }

    public void setRobotVelocityObservation(Twist2d measuredVelocity) {
        this.measuredVelocity = measuredVelocity;
    }

    public Pose2d getFieldToRobot(double timestamp) {
        return fieldToRobot.getSample(timestamp);
    }

    public Pose2d getRobotToTurret(double timestamp) {
        return robotToTurret.getSample(timestamp);
    }

    public Pose2d getFieldToTurret(double timestamp) {
        var ftr = getFieldToRobot(timestamp);
        var rtt = getRobotToTurret(timestamp);
        if (ftr == null || rtt == null) {
            return null;
        }
        return rtt.relativeTo(ftr);
    }

    public Transform2d getTurretToTarget(double timestamp, Pose2d fieldToTarget) {
        var ftTurret = getFieldToTurret(timestamp);
        if (ftTurret == null || fieldToTarget == null) {
            return null;
        }
        return fieldToTarget.minus(ftTurret);
        // X->Y = Z->y - Z->X
    }

    public Twist2d getMeasuredVelocity() {
        return measuredVelocity;
    }

    /**
     * Basically what the goal tracker does is every time the vision system sees a target it adds
     * its position to the goal tracker, if there are previously tracked goals that are close enough
     * using the norm() to the "new" target then it is assumed to be that target is already tracked
     * and we are just updating it's location on the field. If unable to update either because
     * current tracked is too old, or new target is too far away, then assume new target
     */
    /**
     * When we get aiming parameters we need to sort all the tracked goals and get the "best" one
     * using some custom comparator
     */
}
