package team3647.frc2022.constants;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class HoodContants {

    public static final TalonFX kHoodMotor = new TalonFX(GlobalConstants.HoodIds.kMotorId);

    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();
    public static final double encoderTicksToDegrees = 0;
    public static final double maxAngleDegrees = 90;
    public static final double minAngleDegrees = 15;

    static {
        kMasterConfig.slot0.kP = 0;
        kMasterConfig.slot0.kI = 0;
        kMasterConfig.slot0.kD = 0;
        kMasterConfig.slot0.kF = 0;

        kMasterConfig.voltageCompSaturation = 12;

        kHoodMotor.configAllSettings(kMasterConfig);
    }
}
