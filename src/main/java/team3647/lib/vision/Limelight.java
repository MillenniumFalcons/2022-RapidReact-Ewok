package team3647.lib.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.EntryNotification;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import team3647.lib.utils.NamedInt;
import team3647.lib.vision.IVisionCamera.CamMode;
import team3647.lib.vision.IVisionCamera.LEDMode;

public class Limelight implements IVisionCamera {

    // NetworkTable is the class used to grab values from the Limelight Network
    // Table
    private final NetworkTable table;
    private final double[] emptyDoubleArray = {};
    private final double extraLatencySec;
    private final String ip;
    private final CamConstants kCamConstants;

    private double[] xCorners;
    private double[] yCorners;
    private double captureTimestamp = 0.0;
    private VisionPipeline currentPipeline;

    public enum Data {
        VALID_TARGET("tv"),
        X("tx"),
        Y("ty"),
        AREA("ta"),
        SKEW("ts"),
        LATNECY_MS("tl"),
        RAW_CORNERS("tcornxy");

        public final String str;

        Data(String str) {
            this.str = str;
        }
    }

    // used to initalize the main, important things
    public Limelight(String ip, double extraLatencySec, CamConstants camConstants) {
        this(ip, "limelight", extraLatencySec, camConstants);
    }

    public Limelight(String ip, String name, double extraLatencySec, CamConstants camConstants) {
        // initializing the network table to grab values from limelight
        table = NetworkTableInstance.getDefault().getTable(name);
        this.ip = ip;
        this.extraLatencySec = extraLatencySec;
        this.kCamConstants = camConstants;
        table.getEntry(Data.RAW_CORNERS.str)
                .addListener(this::processNTEvent, EntryListenerFlags.kUpdate);
    }

    public synchronized void writeToInputs(VisionInputs inputs) {
        inputs.xCorners = xCorners;
        inputs.yCorners = yCorners;
        inputs.captureTimestamp = captureTimestamp;
    }

    private void processNTEvent(EntryNotification notification) {
        double[] latestRawCorners = getDoubleArray(Data.RAW_CORNERS);
        double timestamp =
                Timer.getFPGATimestamp() - getDouble(Data.LATNECY_MS) / 1000.0 - extraLatencySec;
        synchronized (Limelight.this) {
            captureTimestamp = timestamp;
            xCorners = new double[latestRawCorners.length / 2];
            yCorners = new double[latestRawCorners.length / 2];
            for (int i = 0; i < latestRawCorners.length - 1; i += 2) {
                xCorners[i / 2] = latestRawCorners[i];
                yCorners[i / 2] = latestRawCorners[i + 1];
            }
        }
    }

    @Override
    public void setLED(LEDMode ledMode) {
        set("ledMode", ledMode);
    }

    @Override
    public void setCamMode(CamMode camMode) {
        set("camMode", camMode);
    }

    @Override
    public void setPipeline(VisionPipeline pipeline) {
        this.currentPipeline = pipeline;
        set("pipeline", pipeline);
    }

    private void set(String input, NamedInt input2) {
        table.getEntry(input).setNumber(input2.asInt);
    }

    private double[] getDoubleArray(Data key) {
        return table.getEntry(key.str).getDoubleArray(emptyDoubleArray);
    }

    private double getDouble(Data key) {
        return table.getEntry(key.str).getDouble(22);
    }

    public String toString() {
        return ip;
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
