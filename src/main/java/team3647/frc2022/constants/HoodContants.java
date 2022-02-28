package team3647.frc2022.constants;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import team3647.lib.drivers.LazyTalonFX;
import team3647.lib.team254.util.InterpolatingDouble;
import team3647.lib.team254.util.InterpolatingTreeMap;

public class HoodContants {

    public static final TalonFX kHoodMotor = new LazyTalonFX(GlobalConstants.HoodIds.kMotorId);
    public static final TalonFXInvertType kHoodMotorInvert = TalonFXInvertType.Clockwise;

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();
    public static final double kGearboxReduction = 16 / 48.0 * 24 / 460.0;
    public static final double kFalconPositionToDegrees = kGearboxReduction / 2048.0 * 360;
    public static final double kFalconVelocityToDegpS = kFalconPositionToDegrees * 10;
    public static final double kMaxDegree = 33;
    public static final double kMinDegree = 15;
    public static final double kPosThersholdDeg = 0.5;
    public static final boolean kCurrentLimitingEnable = true;
    public static final double kS = 0.73693;
    public static final double kV = 0.014462;
    public static final double kA = 0.00069253;

    public static final double kMaxVelocityDegPs = 36;
    public static final double kMaxAccelerationDegPss = 36;
    public static final double kMaxVelocityTicks = kMaxVelocityDegPs / kFalconVelocityToDegpS;
    public static final double kMaxAccelerationTicks =
            kMaxAccelerationDegPss / kFalconVelocityToDegpS;

    public static final double kStallCurrent = 10;
    public static final double kContinuousCurrentLimit = 35; // amps
    public static final double kPeakCurrentLimit = 40; // amps
    public static final double kPeakCurrentDuration = 10; // milliseconds
    public static final double kNominalVoltage = 11;

    public static final double[][] kHoodMap = {
        {Units.feetToMeters(2) + GlobalConstants.centerOffsetMeters, 18},
        {Units.feetToMeters(4) + GlobalConstants.centerOffsetMeters, 22.5},
        // {Units.feetToMeters(8) + GlobalConstants.centerOffsetMeters, 27.97},
        // {Units.feetToMeters(12) + GlobalConstants.centerOffsetMeters, 31.97},
        // {Units.feetToMeters(14) + GlobalConstants.centerOffsetMeters, 32},
        // {Units.feetToMeters(16) + GlobalConstants.centerOffsetMeters, 31.97}

        {Units.feetToMeters(8) + GlobalConstants.centerOffsetMeters, 28},
        {Units.feetToMeters(9) + GlobalConstants.centerOffsetMeters, 28},
        {Units.feetToMeters(10) + GlobalConstants.centerOffsetMeters, 29},
        {Units.feetToMeters(11) + GlobalConstants.centerOffsetMeters, 29},
        {Units.feetToMeters(12) + GlobalConstants.centerOffsetMeters, 29},
        {Units.feetToMeters(13) + GlobalConstants.centerOffsetMeters, 30},
        {Units.feetToMeters(14) + GlobalConstants.centerOffsetMeters, 30}
    };

    public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
            kHoodAutoAimMap = new InterpolatingTreeMap<>();

    static {
        kMasterConfig.slot0.kP = 0.21;
        kMasterConfig.slot0.kI = 0;
        kMasterConfig.slot0.kD = 0.2025;
        // kMasterConfig.slot0.integralZone = 1000;
        kMasterConfig.slot0.kF = kV / kNominalVoltage * kFalconVelocityToDegpS * 1023;

        kMasterConfig.voltageCompSaturation = kNominalVoltage;
        kMasterConfig.supplyCurrLimit.enable = kCurrentLimitingEnable;
        kMasterConfig.supplyCurrLimit.currentLimit = kStallCurrent;
        kMasterConfig.supplyCurrLimit.triggerThresholdCurrent = kPeakCurrentLimit;
        kMasterConfig.supplyCurrLimit.triggerThresholdTime = kPeakCurrentDuration;

        // in native units/100ms/s
        kMasterConfig.motionAcceleration = kMaxVelocityTicks;
        // in native units/100ms
        kMasterConfig.motionCruiseVelocity = kMaxAccelerationTicks;

        kHoodMotor.configAllSettings(kMasterConfig, GlobalConstants.kTimeoutMS);
        kHoodMotor.setInverted(kHoodMotorInvert);

        for (double[] pair : kHoodMap) {
            kHoodAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
    }

    public static double getHoodAngle(double range) {
        InterpolatingDouble d = kHoodAutoAimMap.getInterpolated(new InterpolatingDouble(range));

        return d == null ? 16 : MathUtil.clamp(d.value, 15, 33);
    }
}
