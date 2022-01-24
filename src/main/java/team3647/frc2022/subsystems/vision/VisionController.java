package team3647.frc2022.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.inputs.Limelight;
import team3647.lib.inputs.Limelight.Data;

public class VisionController implements PeriodicSubsystem {
    private final PeriodicIO periodicIO = new PeriodicIO();
    private final Limelight ll;
    private final double kTargetHeightMeters;
    private final CamConstants kCamConstants;

    public static class CamConstants {
        public final double netTablename;
        public final double lensHeight;
        public final double capLatencySec;
        public final Rotation2d horizontalToLens;

        public CamConstants(
                double netTablename,
                double lensHeight,
                double capLatencySec,
                Rotation2d horizontalToLens) {
            this.netTablename = netTablename;
            this.lensHeight = lensHeight;
            this.capLatencySec = capLatencySec;
            this.horizontalToLens = horizontalToLens;
        }
    }

    public static class PeriodicIO {
        // inputs
        public boolean validTarget;
        public double xOffset;
        public double yOffset;
        public double latency;
        public double area;
        public double timestamp;

        // outputs
    }

    public VisionController(Limelight ll, double kTargetHeightMeters, CamConstants constants) {
        this.ll = ll;
        this.kTargetHeightMeters = kTargetHeightMeters;
        this.kCamConstants = constants;
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.timestamp = Timer.getFPGATimestamp();
        periodicIO.latency = ll.get(Data.LATNECY);
        periodicIO.xOffset = ll.get(Data.X);
        periodicIO.yOffset = ll.get(Data.Y);
        periodicIO.validTarget = 1.0 == ll.get(Data.VALID_TARGET);
        periodicIO.area = ll.get(Data.AREA);
    }

    public Translation2d getCamToTarget() {
        double denom =
                Math.tan(Math.toRadians(getYOffset()) + kCamConstants.horizontalToLens.getRadians())
                        * Math.cos(Math.toRadians(getXOffset()));
        double num = kTargetHeightMeters - kCamConstants.lensHeight;
        double range = num / denom;
        return new Translation2d();
    }

    public double getLatency() {
        return periodicIO.latency + kCamConstants.capLatencySec;
    }

    public double getTimestamp() {
        return periodicIO.timestamp;
    }

    public double getXOffset() {
        return periodicIO.xOffset;
    }

    public double getYOffset() {
        return periodicIO.yOffset;
    }

    public boolean getValidTarget() {
        return periodicIO.validTarget;
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return null;
    }
}
