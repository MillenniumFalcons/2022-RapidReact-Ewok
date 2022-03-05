package team3647.frc2022.constants;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import team3647.lib.drivers.LazyTalonFX;

public final class ColumnBottomConstants {
    public static final TalonFXInvertType kMasterInverted = TalonFXInvertType.Clockwise;
    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();
    public static final double kS = 0.7;
    public static final double kV = 1.3;
    public static final double kA = 0.001;
    public static final SimpleMotorFeedforward kFeedForward =
            new SimpleMotorFeedforward(kS, kV, kA);

    public static final double kNominalVoltage = 12;
    public static final double kGearboxReduction = 19 / 36.0;

    public static final double kWheelDiameterMeters = 0.0508;
    public static final double kWheelRotationMeters =
            kWheelDiameterMeters * Math.PI * kGearboxReduction;

    public static final double kPosConverstion =
            kWheelRotationMeters / GlobalConstants.kFalconTicksPerRotation;
    public static final double kNativeVelToSurfaceMpS =
            10 * kWheelDiameterMeters / GlobalConstants.kFalconTicksPerRotation;

    public static final TalonFX kColumnMotor =
            new LazyTalonFX(GlobalConstants.ColumnBottomIds.kMotorId);
    public static final DigitalInput kBottomBanner =
            new DigitalInput(GlobalConstants.ColumnBottomIds.kBottomSensorPin);
    public static final DigitalInput kMiddleBanner =
            new DigitalInput(GlobalConstants.ColumnBottomIds.kMiddleSensorPin);
    public static final Solenoid kBallstopperPiston =
            new Solenoid(GlobalConstants.kPCMType, GlobalConstants.ColumnBottomIds.kSolenoidPin);

    static {
        kMasterConfig.slot0.kP = 0;
        kMasterConfig.slot0.kI = 0;
        kMasterConfig.slot0.kD = 0;
        kMasterConfig.slot0.kF = 0;

        kMasterConfig.supplyCurrLimit.enable = true;
        kMasterConfig.supplyCurrLimit.triggerThresholdCurrent = 40;
        kMasterConfig.supplyCurrLimit.triggerThresholdTime = 0.5;
        kMasterConfig.supplyCurrLimit.currentLimit = 15;

        kMasterConfig.voltageCompSaturation = kNominalVoltage;

        kColumnMotor.configAllSettings(kMasterConfig, GlobalConstants.kTimeoutMS);
    }
}
