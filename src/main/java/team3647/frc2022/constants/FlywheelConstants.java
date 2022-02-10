package team3647.frc2022.constants;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import team3647.lib.drivers.LazyTalonFX;

public final class FlywheelConstants {

    public static final TalonFXInvertType kMasterInverted = TalonFXInvertType.Clockwise;
    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final boolean kCurrentLimitingEnable = true;
    public static final double kStallCurrent = 10;
    public static final double kMaxCurrent = 100;
    public static final double kMaxCurrentDurationSec = 1;

    public static final double kS = 0.78509;
    public static final double kV = 0.184;
    public static final double kA = 0.052202;
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

    static {
        kMasterConfig.slot0.kP = 0.37289;
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
        kMaster.setInverted(kMasterInverted);
    }

    private FlywheelConstants() {}
}
