package team3647.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

public class Geometry {
    public static Pose2d poseInverse(Pose2d pose) {
        final Transform2d newTransform =
                new Transform2d(pose.getTranslation(), pose.getRotation()).inverse();
        return new Pose2d(newTransform.getTranslation(), newTransform.getRotation());
    }

    public static Pose2d tranformPosebyPose(Pose2d toTranform, Pose2d transform) {
        final Transform2d transform2d =
                new Transform2d(transform.getTranslation(), transform.getRotation());
        return toTranform.transformBy(transform2d);
    }
}
