package team3647.frc2022.constants;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import team3647.lib.drivers.LazyTalonFX;
import team3647.lib.team254.util.InterpolatingDouble;
import team3647.lib.team254.util.InterpolatingTreeMap;

public final class FlywheelConstants {

    public static final TalonFXInvertType kMasterInverted = TalonFXInvertType.Clockwise;
    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final boolean kCurrentLimitingEnable = true;
    public static final double kStallCurrent = 10;
    public static final double kMaxCurrent = 100;
    public static final double kMaxCurrentDurationSec = 1;

    public static final double kS = 0.537; // 0.57; // 0.74;
    public static final double kV = 0.25086; // 0.2;
    public static final double kA = 0.11684; // 0;
    public static final SimpleMotorFeedforward kFeedForward =
            new SimpleMotorFeedforward(kS, kV, kA);
    public static final double kNominalVoltage = 10;

    public static final TalonFX kMaster = new LazyTalonFX(GlobalConstants.FlywheelIds.kMasterId);
    public static final TalonFX kFollower =
            new LazyTalonFX(GlobalConstants.FlywheelIds.kFollowerId);
    public static final double kGearboxReduction = 36.0 / 24;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kWheelRotationMeters = kWheelDiameterMeters * Math.PI;
    public static final double kNativeVelToSurfaceMpS =
            10 * kWheelRotationMeters / GlobalConstants.kFalconTicksPerRotation * kGearboxReduction;

    public static final double kLowGoalVelocity = 5;
    public static final double kBatterVelocity = 8.5;

    public static final double[][] kFlywheelMap = {
        {Units.feetToMeters(2) + GlobalConstants.centerOffsetMeters, 8.5},
        {Units.feetToMeters(6) + GlobalConstants.centerOffsetMeters, 9.8},
        {Units.feetToMeters(10) + GlobalConstants.centerOffsetMeters, 11},
        {Units.feetToMeters(14) + GlobalConstants.centerOffsetMeters, 12},
        {Units.feetToMeters(18) + GlobalConstants.centerOffsetMeters, 13},
        {Units.feetToMeters(22) + GlobalConstants.centerOffsetMeters, 14.4},
    };

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
            kFlywheelAutoAimMap = new InterpolatingTreeMap<>();
    public static double constantVelocityMpS = 5;

    static {
        kMasterConfig.slot0.kP = 0.14; // 0.17;
        kMasterConfig.slot0.kI = 0;
        kMasterConfig.slot0.kD = 0.1; // 0.08;
        kMasterConfig.slot0.kF = 0;

        kMasterConfig.voltageCompSaturation = kNominalVoltage;
        kMasterConfig.openloopRamp = 0;
        kMasterConfig.supplyCurrLimit.enable = kCurrentLimitingEnable;
        kMasterConfig.supplyCurrLimit.currentLimit = kStallCurrent;
        kMasterConfig.supplyCurrLimit.triggerThresholdCurrent = kMaxCurrent;
        kMasterConfig.supplyCurrLimit.triggerThresholdTime = kMaxCurrentDurationSec;
        kMaster.configAllSettings(kMasterConfig, GlobalConstants.kTimeoutMS);
        kFollower.configAllSettings(kMasterConfig, GlobalConstants.kTimeoutMS);
        kMaster.setInverted(kMasterInverted);

        for (double[] pair : kFlywheelMap) {
            kFlywheelAutoAimMap.put(
                    new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
    }

    public static double getFlywheelRPM(double range) {
        InterpolatingDouble d = kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range));

        return d == null ? 12 : MathUtil.clamp(d.value, 7, 35);
    }

    private FlywheelConstants() {}
}
