package team3647.lib.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import java.util.LinkedList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

// TODO: Finish implementation
public class PhotonVisionCamera implements IVisionCamera {
    private final PhotonCamera camera;
    private final double[] emptyDoubleArray = {};
    private final double extraLatencySec;
    private final CamConstants kCamConstants;

    private double[] xCorners;
    private double[] yCorners;
    private double captureTimestamp = 0.0;
    private VisionPipeline currentPipeline = new VisionPipeline(0, 960, 720);

    public PhotonVisionCamera(String name, double extraLatencySec, CamConstants constants) {
        camera = new PhotonCamera(name);
        this.kCamConstants = constants;
        this.extraLatencySec = extraLatencySec;
        NetworkTableInstance.getDefault()
                .getEntry("/photonvision/" + name + "/latencyMillis")
                .addListener(this::processNTEvent, EntryListenerFlags.kUpdate);
    }

    @Override
    public synchronized void writeToInputs(VisionInputs inputs) {
        inputs.xCorners = xCorners;
        inputs.yCorners = yCorners;
        inputs.captureTimestamp = captureTimestamp;
    }

    private void processNTEvent(EntryNotification notification) {
        PhotonPipelineResult result = camera.getLatestResult();
        double timestamp =
                Timer.getFPGATimestamp() - result.getLatencyMillis() / 1000.0 - extraLatencySec;
        List<Double> xCornersList = new LinkedList<>();
        List<Double> yCornersList = new LinkedList<>();
        for (PhotonTrackedTarget target : result.getTargets()) {
            for (TargetCorner corner : target.getCorners()) {
                xCornersList.add(corner.x);
                yCornersList.add(corner.y);
            }
        }
        synchronized (PhotonVisionCamera.this) {
            captureTimestamp = timestamp;
            xCorners = xCornersList.stream().mapToDouble(Double::doubleValue).toArray();
            yCorners = yCornersList.stream().mapToDouble(Double::doubleValue).toArray();
        }
    }

    @Override
    public void setLED(LEDMode ledMode) {
        // camera.setLED();
    }

    @Override
    public void setCamMode(CamMode camMode) {
        // TODO Auto-generated method stub

    }

    @Override
    public void setPipeline(VisionPipeline pipeline) {
        currentPipeline = pipeline;
    }

    @Override
    public Rotation2d getHorizontalPlaneToLens() {
        return kCamConstants.kHorizontalToLens;
    }

    @Override
    public double getLensHeightMeters() {
        return kCamConstants.kCameraHeightMeters;
    }

    @Override
    public double getVPW() {
        return kCamConstants.kVPW;
    }

    @Override
    public double getVPH() {
        return kCamConstants.kVPH;
    }

    @Override
    public CamConstants getConstants() {
        return kCamConstants;
    }

    @Override
    public VisionPipeline getPipeline() {
        return currentPipeline;
    }
}
