package team3647.frc2022.constants;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public final class VerticalRollersConstants {
    public static final TalonFXInvertType kMasterInverted = TalonFXInvertType.Clockwise;
    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final SimpleMotorFeedforward kFeedForward =
            new SimpleMotorFeedforward(kS, kV, kA);

    public static final double kNominalVoltage = 12;
    public static final double kGearboxReduction = 1;

    public static final double kWheelDiameterMeters = 0.0508;

    public static final double kWheelRotationMeters =
            kWheelDiameterMeters * Math.PI * kGearboxReduction;

    public static final double kPosConverstion =
            kWheelRotationMeters / GlobalConstants.kFalconTicksPerRotation;
    public static final double kNativeVelToSurfaceMpS =
            10 * kWheelDiameterMeters / GlobalConstants.kFalconTicksPerRotation;

    public static final TalonFX kColumnMotor =
            new TalonFX(GlobalConstants.VerticalRollersIds.kMotorId);

    static {
        kMasterConfig.slot0.kP = 0;
        kMasterConfig.slot0.kI = 0;
        kMasterConfig.slot0.kD = 0;
        kMasterConfig.slot0.kF = 0;

        kMasterConfig.voltageCompSaturation = kNominalVoltage;

        kColumnMotor.configAllSettings(kMasterConfig);
    }
}