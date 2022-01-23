package team3647.lib.vision;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.LinkedList;
import java.util.List;

public class MultiTargetTracker {
    private final List<TrackedTarget> trackedTargets = new LinkedList<>();
    private int nextId = 0;

    public void update(double timestamp, Iterable<Pose2d> fieldToGoals) {
        for (Pose2d target : fieldToGoals) {
            boolean foundCorrespondingTarget = false;
            for (TrackedTarget tracked : trackedTargets) {
                if (foundCorrespondingTarget) {
                    tracked.removeOldObservations();
                } else {
                    foundCorrespondingTarget = tracked.attemptUpdate(timestamp, target);
                }
            }
            if (!foundCorrespondingTarget) {
                trackedTargets.add(new TrackedTarget(timestamp, nextId++, target));
            }
        }
        removeDeadTargets();
    }

    private void removeDeadTargets() {
        trackedTargets.removeIf(tracked -> !tracked.isAlive());
    }

    public boolean hasTracks() {
        return !trackedTargets.isEmpty();
    }

    public List<TrackedTarget> getTrackedTargets() {
        return trackedTargets;
    }
}
