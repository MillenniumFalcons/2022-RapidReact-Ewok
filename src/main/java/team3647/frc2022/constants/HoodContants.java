package team3647.frc2022.constants;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import team3647.lib.drivers.LazyTalonFX;

public class HoodContants {

    public static final TalonFX kHoodMotor = new LazyTalonFX(GlobalConstants.HoodIds.kMotorId);
    public static final TalonFXInvertType kHoodMotorInvert = TalonFXInvertType.Clockwise;

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();
    public static final double degPerTick = /*((40 * 2048) / 360);*/ (360.0 / (40 * 2048));
    public static final double maxAngleDegrees = 45;
    public static final double minAngleDegrees = 15;
    public static final boolean kCurrentLimitingEnable = true;

    public static final double maxVelocityDegPerSec = 36;
    public static final double maxAccelerationDegPerSecSqr = 36;
    public static final double maxVelocityTicksPerHundredMS =
            (maxVelocityDegPerSec / degPerTick) * (1 / 10);
    public static final double maxAccelerationTicksPerHundredMS =
            (maxAccelerationDegPerSecSqr / degPerTick) * (1 / 100);

    public static final double kStallCurrent = 10;
    public static final double kContinuousCurrentLimit = 35; // amps
    public static final double kPeakCurrentLimit = 40; // amps
    public static final double kPeakCurrentDuration = 10; // milliseconds
    public static final double kMaxVoltage = 12.0;

    static {
        kMasterConfig.slot0.kP = 0.7;
        kMasterConfig.slot0.kI = 0;
        kMasterConfig.slot0.kD = 20;
        kMasterConfig.slot0.kF = 0;

        kMasterConfig.voltageCompSaturation = 12;
        kMasterConfig.supplyCurrLimit.enable = kCurrentLimitingEnable;
        kMasterConfig.supplyCurrLimit.currentLimit = kStallCurrent;
        kMasterConfig.supplyCurrLimit.triggerThresholdCurrent = kPeakCurrentLimit;
        kMasterConfig.supplyCurrLimit.triggerThresholdTime = kPeakCurrentDuration;

        // in native units/100ms^2
        kMasterConfig.motionAcceleration = (maxAccelerationDegPerSecSqr / 100) / (degPerTick);
        // in native units/100ms
        kMasterConfig.motionCruiseVelocity = (maxVelocityDegPerSec / 10) / degPerTick;

        kHoodMotor.configAllSettings(kMasterConfig, GlobalConstants.kTimeoutMS);
        kHoodMotor.setInverted(kHoodMotorInvert);
    }
}
