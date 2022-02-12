package team3647.frc2022.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import team3647.lib.vision.AimingParameters;
import team3647.lib.vision.MultiTargetTracker;
import team3647.lib.vision.TrackedTarget;
import team3647.lib.vision.TrackedTarget.TrackedTargetComparator;

public class FlightDeck {
    private final RobotTracker robotTracker;
    private final MultiTargetTracker targetTracker;
    private final Pose2d kTurretToCamFixed;
    private static final Translation2d turretToCam = new Translation2d(Units.inchesToMeters(7), 0);
    public static double maxAge;

    public FlightDeck(
            RobotTracker robotTracker, MultiTargetTracker targetTracker, Pose2d kTurretToCamFixed) {
        this.robotTracker = robotTracker;
        this.targetTracker = targetTracker;
        this.kTurretToCamFixed = kTurretToCamFixed;
    }

    public synchronized void addVisionObservation(double timestamp, Translation2d camToGoal) {
        Pose2d fieldToTurret = robotTracker.getFieldToTurret(timestamp);
        targetTracker.update(
                timestamp,
                List.of(
                        new Pose2d(camToGoal, new Rotation2d())
                                .relativeTo(kTurretToCamFixed)
                                .relativeTo(fieldToTurret)));
    }

    public synchronized AimingParameters getAimingParameters(int lastTargetId) {
        List<TrackedTarget> targets = targetTracker.getTrackedTargets();
        if (targets.isEmpty()) {
            return null;
        }
        double timestamp = Timer.getFPGATimestamp();
        TrackedTargetComparator comparator =
                new TrackedTargetComparator(0.0, 10.0, timestamp, 100.0, lastTargetId);
        targets.sort(comparator);
        TrackedTarget bestTarget = targets.get(0);
        for (TrackedTarget target : targets) {
            if (target.getLatestTimestamp() > timestamp - maxAge) {
                bestTarget = target;
            }
        }
        return new AimingParameters(
                bestTarget.id,
                robotTracker.getFieldToTurret(timestamp),
                bestTarget.getSmoothedPosition(),
                bestTarget.getLatestTimestamp(),
                bestTarget.getStability());
    }
}
