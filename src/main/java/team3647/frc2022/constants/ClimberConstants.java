package team3647.frc2022.constants;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.Solenoid;

public final class ClimberConstants {
    public static final TalonFX kLeftMotor = new TalonFX(GlobalConstants.ClimberIds.kLeftMotorId);
    public static final TalonFX kRightMotor = new TalonFX(GlobalConstants.ClimberIds.kRightMotorId);
    public static final TalonFXInvertType kLeftInvert = TalonFXInvertType.CounterClockwise;
    public static final TalonFXInvertType kRightInvert = TalonFXInvertType.Clockwise;

    public static final Solenoid kPivotPistons =
            new Solenoid(GlobalConstants.kPCMType, GlobalConstants.ClimberIds.kSolenoidPin);

    public static final double kNominalVoltage = 12;
    public static final double kVoltageToHoldRobot = 5;

    public static final double kGearboxReduction = 1;
    public static final double kOutputRotationDiameter = 0.0508;
    public static final double kOutputRotationMeters =
            kOutputRotationDiameter * Math.PI * kGearboxReduction;
    public static final double kPosConverstion =
            kOutputRotationMeters / GlobalConstants.kFalconTicksPerRotation;
    public static final double kNativeVelToMpS =
            10 * kOutputRotationMeters / GlobalConstants.kFalconTicksPerRotation;
    public static final double kMaxLengthAngled = 2;
    public static final double kMaxLengthStraight = 1.75;

    public static final TalonFXConfiguration kLeftConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration kRightConfig = new TalonFXConfiguration();

    static {
        kLeftConfig.slot0.kP = 0;
        kLeftConfig.slot0.kI = 0;
        kLeftConfig.slot0.kD = 0;
        kLeftConfig.slot0.kF = 0;

        kLeftConfig.voltageCompSaturation = kNominalVoltage;

        kLeftMotor.configAllSettings(kLeftConfig);
        kLeftMotor.setInverted(kLeftInvert);
    }

    static {
        kRightConfig.slot0.kP = 0;
        kRightConfig.slot0.kI = 0;
        kRightConfig.slot0.kD = 0;
        kRightConfig.slot0.kF = 0;

        kRightConfig.voltageCompSaturation = kNominalVoltage;

        kRightMotor.configAllSettings(kLeftConfig);
        kLeftMotor.setInverted(kRightInvert);
    }
}
