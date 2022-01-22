package team3647.lib.wpi;

import edu.wpi.first.math.geometry.Pose2d;

public class TimestampedPose {
    public final Pose2d pose;
    public final double timestamp;

    public TimestampedPose(Pose2d pose, double timestamp) {
        this.pose = pose;
        this.timestamp = timestamp;
    }
}
