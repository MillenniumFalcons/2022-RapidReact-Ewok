package team3647.frc2022.subsystems;

import java.util.Objects;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.team254.util.MovingAverage;
import team3647.lib.wpi.HALMethods;

public class VisionController implements PeriodicSubsystem {

    public static class CamConstants {
        public final double kGoalHeight;
        public final double kCameraHeight;
        public final double kCameraAngle;
        public final double kImageCaptureLatency;

        public CamConstants(
                double goalHeight,
                double cameraHeight,
                double cameraAngle,
                double imageCaptureLatency) {
            this.kGoalHeight = goalHeight;
            this.kCameraHeight = cameraHeight;
            this.kCameraAngle = cameraAngle;
            this.kImageCaptureLatency = imageCaptureLatency;
        }
    }

    public static class PeriodicIO {
        public boolean validTarget;
        public double x;
        public double y;
        public double area;
        public double skew;
        public double latency;
        public double range;

        public Pipeline pipeline = Pipeline.DEFAULT;
    }

    private final PhotonCamera photonCamera;
    private final CamConstants m_constants;
    private boolean outputsHaveChanged = false;
    private PeriodicIO periodicIO = new PeriodicIO();

    private MovingAverage xAverage = new MovingAverage(8);
    private MovingAverage yAverage = new MovingAverage(8);
    private MovingAverage areaAverage = new MovingAverage(8);
    private MovingAverage skewAverage = new MovingAverage(8);
    private MovingAverage rangeAverage = new MovingAverage(8);

    public enum Pipeline {
        DEFAULT(0);

        public final int id;

        Pipeline(int id) {
            this.id = id;
        }
    }

    public VisionController(String camName, CamConstants constants) {
        Objects.requireNonNull(camName);
        Objects.requireNonNull(constants);
        this.photonCamera = new PhotonCamera(camName);
        m_constants = constants;
        setPipeline(Pipeline.DEFAULT);
    }

    @Override
    public void readPeriodicInputs() {
        var camResult = photonCamera.getLatestResult();
        periodicIO.validTarget = camResult.hasTargets();

        if (periodicIO.validTarget) {
            periodicIO.x = camResult.getBestTarget().getYaw();
            xAverage.add(periodicIO.x);

            periodicIO.y = camResult.getBestTarget().getPitch();
            yAverage.add(periodicIO.y);

            periodicIO.area = camResult.getBestTarget().getArea();
            areaAverage.add(periodicIO.area);

            periodicIO.skew = camResult.getBestTarget().getSkew();
            skewAverage.add(periodicIO.skew);
        }
        periodicIO.latency = camResult.getLatencyMillis() + m_constants.kImageCaptureLatency;

        periodicIO.range = calculateRange(getFilteredPitch());
        rangeAverage.add(periodicIO.range);
    }

    @Override
    public void writePeriodicOutputs() {
        if (outputsHaveChanged) {
            photonCamera.setDriverMode(false);
            photonCamera.setLED(VisionLEDMode.kDefault);
            photonCamera.setPipelineIndex(periodicIO.pipeline.id);
            HALMethods.sendDSWarning("Camera changed modes!");
            outputsHaveChanged = false;
        }
    }

    @Override
    public void periodic() {
        readPeriodicInputs();
        writePeriodicOutputs();
    }

    public double getDistance() {
        return periodicIO.range;
    }

    public double getFilteredDistance() {
        return rangeAverage.getAverage();
    }

    public double getYaw() {
        return periodicIO.x;
    }

    public double getFilteredYaw() {
        return xAverage.getAverage();
    }

    public boolean isValid() {
        return periodicIO.validTarget;
    }

    public double getPitch() {
        return periodicIO.y;
    }

    public double getFilteredPitch() {
        return yAverage.getAverage();
    }

    private double calculateRange(double yDegrees) {
        return (m_constants.kGoalHeight - m_constants.kCameraHeight)
                / Math.tan(Math.toRadians(yDegrees + m_constants.kCameraAngle));
    }

    public void setPipeline(Pipeline pipeline) {
        periodicIO.pipeline = pipeline;
        outputsHaveChanged = true;
    }

    @Override
    public String getName() {
        return "VisionController";
    }
}
