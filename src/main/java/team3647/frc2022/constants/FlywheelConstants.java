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

    public static final double kS = 0.23; // 0.78509;
    public static final double kV = 0.23; // 0.184;
    public static final double kA = 0.28;
    public static final SimpleMotorFeedforward kFeedForward =
            new SimpleMotorFeedforward(kS, kV, kA);
    public static final double kNominalVoltage = 10;

    public static final TalonFX kMaster = new LazyTalonFX(GlobalConstants.FlywheelIds.kMasterId);
    public static final TalonFX kFollower =
            new LazyTalonFX(GlobalConstants.FlywheelIds.kFollowerId);
    public static final double kGearboxReduction = 2;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kWheelRotationMeters = kWheelDiameterMeters * Math.PI;
    public static final double kNativeVelToSurfaceMpS =
            10 * kWheelRotationMeters / GlobalConstants.kFalconTicksPerRotation * kGearboxReduction;

    public static final double[][] kFlywheelMap = {
        {Units.feetToMeters(2) + GlobalConstants.centerOffsetMeters, 14},
        {Units.feetToMeters(4) + GlobalConstants.centerOffsetMeters, 16},
        // {Units.feetToMeters(8) + GlobalConstants.centerOffsetMeters, 17},
        // {Units.feetToMeters(12) + GlobalConstants.centerOffsetMeters, 20.5},
        // {Units.feetToMeters(14) + GlobalConstants.centerOffsetMeters, 22.5},
        // {Units.feetToMeters(16) + GlobalConstants.centerOffsetMeters, 24.3}

        {Units.feetToMeters(8) + GlobalConstants.centerOffsetMeters, 17.5},
        {Units.feetToMeters(9) + GlobalConstants.centerOffsetMeters, 18},
        {Units.feetToMeters(10) + GlobalConstants.centerOffsetMeters, 19},
        {Units.feetToMeters(11) + GlobalConstants.centerOffsetMeters, 20},
        {Units.feetToMeters(12) + GlobalConstants.centerOffsetMeters, 20},
        {Units.feetToMeters(13) + GlobalConstants.centerOffsetMeters, 21},
        {Units.feetToMeters(14) + GlobalConstants.centerOffsetMeters, 22}
    };

    public static final double[][] kVoltageMap = {
        {2.21, 1},
        {5, 1.27},
        {6, 1.45},
        {7, 1.55},
        {8, 1.7},
        {9, 1.85},
        {10, 2.13},
        {11, 2.3},
        {12, 2.5},
        {13, 2.7},
        {14, 2.85},
        {15, 3.09},
        {16, 3.25},
        {17, 3.45},
        {18, 3.68},
        {19, 3.85},
        {20, 4},
        {21, 4.25},
        {22, 4.45},
        {23, 4.65},
        {24, 4.8},
        {25, 4.95},
        {26, 5.13},
        {27, 5.3},
        {28, 5.47},
        {29, 5.7},
        {30, 5.87},
        {31, 6.1},
        {32, 6.2},
        {33, 6.37},
        {34, 6.54},
        {35, 6.71},
        {36, 6.87},
        {37, 7},
        {38, 7.17},
        {39, 7.34},
        {40, 7.51},
        {41, 7.68}
    };
    public static final double kBatterVelocity = 18;
    public static final double kLowGoalVelocity = 7;

    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
            kFlywheelAutoAimMap = new InterpolatingTreeMap<>();
    public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kFlywheelVoltage =
            new InterpolatingTreeMap<>();
    public static double constantVelocityMpS = kLowGoalVelocity;

    static {
        kMasterConfig.slot0.kP = 0.5;
        kMasterConfig.slot0.kI = 0;
        kMasterConfig.slot0.kD = 0;
        kMasterConfig.slot0.kF = 0;

        kMasterConfig.voltageCompSaturation = kNominalVoltage;
        kMasterConfig.openloopRamp = 0;
        kMasterConfig.supplyCurrLimit.enable = kCurrentLimitingEnable;
        kMasterConfig.supplyCurrLimit.currentLimit = kStallCurrent;
        kMasterConfig.supplyCurrLimit.triggerThresholdCurrent = kMaxCurrent;
        kMasterConfig.supplyCurrLimit.triggerThresholdTime = kMaxCurrentDurationSec;
        kMaster.configAllSettings(kMasterConfig, GlobalConstants.kTimeoutMS);
        kFollower.configAllSettings(kMasterConfig, GlobalConstants.kTimeoutMS);
        kMaster.enableVoltageCompensation(true);
        kFollower.enableVoltageCompensation(true);
        kMaster.setInverted(kMasterInverted);

        for (double[] pair : kFlywheelMap) {
            kFlywheelAutoAimMap.put(
                    new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }

        for (double[] pair : kVoltageMap) {
            kFlywheelVoltage.put(
                    new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
    }

    public static double getFlywheelRPM(double range) {
        InterpolatingDouble d = kFlywheelAutoAimMap.getInterpolated(new InterpolatingDouble(range));

        return d == null ? 6000 : MathUtil.clamp(d.value, 14, 35);
    }

    public static double getFlywheelVoltage(double velocity) {
        InterpolatingDouble d = kFlywheelVoltage.getInterpolated(new InterpolatingDouble(velocity));
        return d == null ? kFeedForward.calculate(velocity) : d.value;
    }

    private FlywheelConstants() {}
}
