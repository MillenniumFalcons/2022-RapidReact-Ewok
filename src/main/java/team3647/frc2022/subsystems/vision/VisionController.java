package team3647.frc2022.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BiConsumer;
import org.ejml.simple.UnsupportedOperation;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.vision.CircleFitter;
import team3647.lib.vision.IVisionCamera;
import team3647.lib.vision.IVisionCamera.CamConstants;
import team3647.lib.vision.IVisionCamera.VisionInputs;
import team3647.lib.vision.IVisionCamera.VisionPipeline;
import team3647.lib.vision.IVisionCamera.VisionPoint;
import team3647.lib.vision.VisionTargetConstants;

public class VisionController implements PeriodicSubsystem {
    private final IVisionCamera camera;
    private final VisionTargetConstants targetConstants;
    private final BiConsumer<Double, Translation2d> translationConsumer;
    private final PeriodicIO periodicIO;
    private static final Rotation2d PiOver2 = new Rotation2d(Math.PI / 2.0);
    private static final double kCircleFitPrecision = 0.01;
    private static final double kNetworklatency = 0.06;

    private static final class PeriodicIO {
        public final VisionInputs inputs = new VisionInputs();
        public double lastTimestamp = 0.0;
    }

    public VisionController(
            IVisionCamera camera,
            VisionTargetConstants targetConstants,
            BiConsumer<Double, Translation2d> translationConsumer) {
        this.camera = camera;
        this.targetConstants = targetConstants;
        this.translationConsumer = translationConsumer;
        this.periodicIO = new PeriodicIO();
    }

    @Override
    public void readPeriodicInputs() {
        camera.writeToInputs(periodicIO.inputs);
        if (periodicIO.lastTimestamp == periodicIO.inputs.captureTimestamp) {
            return;
        }
        periodicIO.lastTimestamp = periodicIO.inputs.captureTimestamp;
        int targetCount = periodicIO.inputs.xCorners.length / targetConstants.kPointsPerTarget;
        if (targetCount < targetConstants.kMinTargetCount) {
            return;
        }
        List<Translation2d> camToTargetTranslations = new LinkedList<>();
        // accessing the arrays as if they are 2d linear (c 2d arrays)
        for (int targetIndex = 0; targetIndex < targetCount; targetIndex++) {
            VisionPoint[] corners = new VisionPoint[targetConstants.kPointsPerTarget];
            double totalX = 0.0;
            double totalY = 0.0;
            for (int cornerIndex = 0;
                    cornerIndex < targetConstants.kPointsPerTarget;
                    cornerIndex++) {
                int idx = targetIndex * targetCount + cornerIndex;
                corners[cornerIndex] =
                        new VisionPoint(
                                periodicIO.inputs.xCorners[idx], periodicIO.inputs.yCorners[idx]);
                totalX += periodicIO.inputs.xCorners[idx];
                totalY += periodicIO.inputs.yCorners[idx];
            }
            VisionPoint targetAvg =
                    new VisionPoint(
                            totalX / targetConstants.kPointsPerTarget,
                            totalY / targetConstants.kPointsPerTarget);

            // Makes the top corners first in the array
            sortCorners(corners, targetAvg);
            int i;
            // top corners
            for (i = 0; i < 4; i++) {
                // uses the top target height for the first 2 elements, and the bottom target height
                // for the last two elements;
                double targetHeight =
                        i < 2
                                ? targetConstants.kTopTargetHeightMeters
                                : targetConstants.kBottomTargetHeightMeters;
                Translation2d camToTarget =
                        solveTranslationToTarget(
                                corners[i],
                                targetHeight,
                                camera.getConstants(),
                                camera.getPipeline());
                if (camToTarget != null) {
                    camToTargetTranslations.add(camToTarget);
                }
            }
        }

        if (camToTargetTranslations.size()
                < targetConstants.kMinTargetCount * targetConstants.kPointsPerTarget) {
            return;
        }
        synchronized (translationConsumer) {
            translationConsumer.accept(
                    periodicIO.inputs.captureTimestamp - kNetworklatency,
                    CircleFitter.fit(
                            targetConstants.kTargetDiameterMeters / 2.0,
                            camToTargetTranslations,
                            kCircleFitPrecision));
        }
    }

    /**
     * ONLY for 4 corners
     *
     * @param corners
     * @param average
     */
    static void sortCorners(VisionPoint[] corners, VisionPoint average) {
        if (corners.length != 4) {
            throw new UnsupportedOperation("Corners needs exactly 4 elements");
        }
        double minAngleInPosDirection = Math.PI;
        double maxAngleInNegDirection = -Math.PI;
        int topLeftIdx = 0;
        int topRightIdx = 1; // garbage values
        int bottomIdx1 = 2;
        int bottomIdx2 = 3;
        for (int i = 0; i < corners.length; i++) {
            VisionPoint corner = corners[i];
            // flip y because 0,0 is the top in the picture
            double angleFromCenterToCorner =
                    new Rotation2d(corner.x - average.x, average.y - corner.y)
                            .minus(PiOver2)
                            .getRadians();
            if (angleFromCenterToCorner > 0
                    && angleFromCenterToCorner
                            < minAngleInPosDirection) { // One of the left corners
                minAngleInPosDirection = angleFromCenterToCorner;
                bottomIdx1 = topLeftIdx;
                topLeftIdx = i;
            } else if (angleFromCenterToCorner > maxAngleInNegDirection) {
                maxAngleInNegDirection = angleFromCenterToCorner;
                bottomIdx2 = topRightIdx;
                topRightIdx = i;
            }
        }

        var topLeftCorner = corners[topLeftIdx];
        var topRightCorner = corners[topRightIdx];
        var bottomCorner1 = corners[bottomIdx1];
        var bottomCorner2 = corners[bottomIdx2];
        corners[0] = topLeftCorner;
        corners[1] = topRightCorner;
        corners[2] = bottomCorner1;
        corners[3] = bottomCorner2;
    }

    public static Translation2d solveTranslationToTarget(
            VisionPoint corner,
            double targetHeightMeters,
            CamConstants camConstants,
            VisionPipeline camPipeline) {
        double yPixels = corner.x;
        double zPixels = corner.y;
        // robot frame, y is left right, z is up down, x is front back
        // Limlieght additional theory
        double halfWidth = camPipeline.width / 2.0;
        double halfHeight = camPipeline.height / 2.0;
        double nY = -(yPixels - halfWidth) / halfWidth;
        double nZ = -(zPixels - halfHeight) / halfHeight;
        Translation2d xzPlaneTranslation =
                new Translation2d(1.0, camConstants.kVPH / 2.0 * nZ)
                        .rotateBy(camConstants.kHorizontalToLens);
        double x = xzPlaneTranslation.getX();
        double y = camConstants.kVPW / 2.0 * nY;
        // This plane is the XZ plane, so the y component of the translation is actually Z
        double z = xzPlaneTranslation.getY();

        double heightDiff = targetHeightMeters / camConstants.kCameraHeightMeters;
        if (z < 0.0 != heightDiff < 0.0) {
            return null;
        }
        double scale = heightDiff / z;
        double range = Math.hypot(x, y) * scale;
        Rotation2d angleToTarget = new Rotation2d(x, y);

        return new Translation2d(range * angleToTarget.getCos(), range * angleToTarget.getSin());
    }

    @Override
    public String getName() {
        return "VisionController";
    }
}
