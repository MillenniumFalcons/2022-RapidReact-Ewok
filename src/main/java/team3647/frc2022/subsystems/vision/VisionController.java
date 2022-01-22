package team3647.frc2022.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.inputs.Limelight;
import team3647.lib.inputs.Limelight.Data;
import team3647.lib.wpi.TimestampedPose;

public class VisionController implements PeriodicSubsystem {
    private final PeriodicIO periodicIO = new PeriodicIO();
    private final Limelight ll;
    private final double kTargetHeightMeters;

    public static class CamConstants {
        public final double netTablename;
        public final double lensHeight;
        public final double capLatencySec;
        public final Rotation2d horizontalToLens;
        public final Pose2d turretToLens;

        public CamConstants(
                double netTablename,
                double lensHeight,
                double capLatencySec,
                Rotation2d horizontalToLens,
                Pose2d turretToLens) {
            this.netTablename = netTablename;
            this.lensHeight = lensHeight;
            this.capLatencySec = capLatencySec;
            this.horizontalToLens = horizontalToLens;
            this.turretToLens = turretToLens;
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

    public VisionController(Limelight ll, double kTargetHeightMeters) {
        this.ll = ll;
        this.kTargetHeightMeters = kTargetHeightMeters;
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

    public Optional<TimestampedPose> getCamToTarget() {
        return Optional.empty();
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return null;
    }
}
