package team3647.frc2022.constants;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import team3647.lib.drivers.LazyTalonFX;
import team3647.lib.team254.util.InterpolatingDouble;
import team3647.lib.team254.util.InterpolatingTreeMap;

public class HoodContants {

    public static final TalonFX kHoodMotor = new LazyTalonFX(GlobalConstants.HoodIds.kMotorId);
    public static final TalonFXInvertType kHoodMotorInvert = TalonFXInvertType.Clockwise;
    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();
    public static final double kGearboxReduction = 10.0 / 50 * 28.0 / 472;
    // 280 / 23600 = 28 / 2360 = 7/590
    public static final double kFalconPositionToDegrees = kGearboxReduction / 2048.0 * 360;
    public static final double kFalconVelocityToDegpS = kFalconPositionToDegrees * 10;
    public static final double kMaxDegree = 40;
    public static final double kMinDegree = 15;
    public static final double kBatterAngle = 15;
    public static final double kLowGoalAngle = 39;
    public static final double kPosThersholdDeg = 0.5;
    public static final boolean kCurrentLimitingEnable = false;
    public static final double kS = 0.451; // 0.85317;
    public static final double kV = 0.03; // 0.00043578;
    public static final double kA = 0.0008;
    public static final double kCos = 0;
    public static final ArmFeedforward kArmFeedforward = new ArmFeedforward(kS, kCos, kV);

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

    public static final double[][] kHoodMap1 = {
        {Units.feetToMeters(4) + GlobalConstants.kCenterOffsetMeters, 18.75},
        // {Units.feetToMeters(4.5) + GlobalConstants.kCenterOffsetMeters, 9.2},
        {Units.feetToMeters(5) + GlobalConstants.kCenterOffsetMeters, 25},
        {Units.feetToMeters(5.5) + GlobalConstants.kCenterOffsetMeters, 26},
        {Units.feetToMeters(6) + GlobalConstants.kCenterOffsetMeters, 27.5},
        {Units.feetToMeters(6.5) + GlobalConstants.kCenterOffsetMeters, 29},
        {Units.feetToMeters(7) + GlobalConstants.kCenterOffsetMeters, 32},
        {Units.feetToMeters(7.5) + GlobalConstants.kCenterOffsetMeters, 32.1},
        {Units.feetToMeters(8) + GlobalConstants.kCenterOffsetMeters, 33},
        {Units.feetToMeters(8.5) + GlobalConstants.kCenterOffsetMeters, 33.75},
        {Units.feetToMeters(9) + GlobalConstants.kCenterOffsetMeters, 34.5},
        {Units.feetToMeters(9.5) + GlobalConstants.kCenterOffsetMeters, 36},
        {Units.feetToMeters(10) + GlobalConstants.kCenterOffsetMeters, 38},
        {Units.feetToMeters(10.5) + GlobalConstants.kCenterOffsetMeters, 38},
        {Units.feetToMeters(11) + GlobalConstants.kCenterOffsetMeters, 39},
        {Units.feetToMeters(11.5) + GlobalConstants.kCenterOffsetMeters, 39},
        {Units.feetToMeters(12) + GlobalConstants.kCenterOffsetMeters, 39},
        {Units.feetToMeters(12.5) + GlobalConstants.kCenterOffsetMeters, 39},
        {Units.feetToMeters(14) + GlobalConstants.kCenterOffsetMeters, 39},
        {Units.feetToMeters(16) + GlobalConstants.kCenterOffsetMeters, 39},
        {Units.feetToMeters(18) + GlobalConstants.kCenterOffsetMeters, 39},
        {Units.feetToMeters(20) + GlobalConstants.kCenterOffsetMeters, 39},
        {Units.feetToMeters(22) + GlobalConstants.kCenterOffsetMeters, 39},
        {Units.feetToMeters(24) + GlobalConstants.kCenterOffsetMeters, 39},
    };
    public static final double[][] kHoodMap = {
        {2.2, 25},
        {2.6, 28},
        {3, 31},
        {3.77, 35},
        {4.15, 38},
        {4.55, 39}
    };

    public static final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
            kHoodAutoAimMap = new InterpolatingTreeMap<>();

    static {
        kMasterConfig.slot0.kP = 0.12;
        kMasterConfig.slot0.kI = 0;
        kMasterConfig.slot0.kD = 1; // 0.45;
        // kMasterConfig.slot0.allowableClosedloopError = 35; // 29;

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
        kHoodMotor.enableVoltageCompensation(true);

        for (double[] pair : kHoodMap) {
            kHoodAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
    }

    public static double getHoodAngle1(double range) {
        InterpolatingDouble d = kHoodAutoAimMap.getInterpolated(new InterpolatingDouble(range));

        return d == null ? 19 : MathUtil.clamp(d.value, kMinDegree, kMaxDegree);
    }
}
