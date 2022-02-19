package team3647.lib.tracking;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.wpilibj.Notifier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Inspired by (basically copied lol) 254 RobotState class */
public class RobotTracker {
    private final double kBufferLengthSeconds;

    private final TimeInterpolatableBuffer<Pose2d> fieldToRobot;
    private final TimeInterpolatableBuffer<Pose2d> robotToTurret;
    private final Translation2d kRobotToTurretFixed;
    private final Supplier<Pose2d> drivetrainGetPose;
    private Pose2d previousPose;
    private final DoubleSupplier drivetrainGetPoseTS;
    private final Supplier<Rotation2d> turretGetRotation;
    private final DoubleSupplier turretGetRotationTS;

    private final Notifier drivetrainUpdate;
    private final Notifier turretUpdate;

    private Twist2d measuredVelocity = new Twist2d();

    public RobotTracker(
            double bufferLengthSeconds,
            Translation2d robotToTurretFixed,
            Supplier<Pose2d> drivetrainGetPose,
            DoubleSupplier drivetrainGetPoseTS,
            Supplier<Rotation2d> turretGetRotation,
            DoubleSupplier turretGetRotationTS) {
        this.kBufferLengthSeconds = bufferLengthSeconds;
        this.kRobotToTurretFixed = robotToTurretFixed;
        fieldToRobot = TimeInterpolatableBuffer.createBuffer(kBufferLengthSeconds);
        robotToTurret = TimeInterpolatableBuffer.createBuffer(kBufferLengthSeconds);
        this.drivetrainGetPose = drivetrainGetPose;
        this.drivetrainGetPoseTS = drivetrainGetPoseTS;
        this.turretGetRotation = turretGetRotation;
        this.turretGetRotationTS = turretGetRotationTS;
        this.drivetrainUpdate = new Notifier(this::addDrivetrainObservation);
        this.turretUpdate = new Notifier(this::addTurretObservation);
        previousPose = drivetrainGetPose.get();
    }

    public void stopTracking() {
        drivetrainUpdate.stop();
        turretUpdate.stop();
        this.fieldToRobot.clear();
        this.robotToTurret.clear();
    }

    public synchronized void addTurretObservation() {
        addTurretRotationObservation(turretGetRotationTS.getAsDouble(), turretGetRotation.get());
    }

    public synchronized void addDrivetrainObservation() {
        addFieldToRobotObservation(drivetrainGetPoseTS.getAsDouble(), drivetrainGetPose.get());
    }

    public synchronized void addFieldToRobotObservation(double timestamp, Pose2d observation) {
        setRobotVelocityObservation(this.previousPose.log(observation));
        this.previousPose = observation;
        fieldToRobot.addSample(timestamp, observation);
    }

    public synchronized void addTurretRotationObservation(
            double timestamp, Rotation2d observation) {
        robotToTurret.addSample(timestamp, new Pose2d(kRobotToTurretFixed, observation));
    }

    public synchronized void setRobotVelocityObservation(Twist2d measuredVelocity) {
        this.measuredVelocity = measuredVelocity;
    }

    public synchronized Pose2d getFieldToRobot(double timestamp) {
        return fieldToRobot.getSample(timestamp);
    }

    public synchronized Pose2d getRobotToTurret(double timestamp) {
        return robotToTurret.getSample(timestamp);
    }

    public Pose2d getFieldToTurret(double timestamp) {
        var ftr = getFieldToRobot(timestamp);
        // System.out.println("FTR: " + ftr);
        if (ftr == null) {
            System.out.println("FTR NULL");
        }
        var rtt = getRobotToTurret(timestamp);
        // System.out.println("FTR: " + rtt);
        if (rtt == null) {
            System.out.println("RTT NULL");
        }
        if (ftr == null || rtt == null) {
            return null;
        }
        var ftt =
                new Transform2d(new Pose2d(), ftr)
                        .plus(
                                new Transform2d(
                                        new Pose2d(new Translation2d(), ftr.getRotation()), rtt));
        return new Pose2d(ftt.getTranslation(), ftt.getRotation());
    }

    public Transform2d getTurretToTarget(double timestamp, Pose2d fieldToTarget) {
        var ftTurret = getFieldToTurret(timestamp);
        if (ftTurret == null || fieldToTarget == null) {
            return null;
        }
        return fieldToTarget.minus(ftTurret);
        // X->Y = Z->y - Z->X
    }

    public synchronized Twist2d getMeasuredVelocity() {
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
