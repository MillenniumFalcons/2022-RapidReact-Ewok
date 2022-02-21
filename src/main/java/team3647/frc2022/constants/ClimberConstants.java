package team3647.frc2022.constants;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Solenoid;
import team3647.lib.drivers.LazyTalonFX;

public final class ClimberConstants {
    public static final TalonFX kLeftMotor =
            new LazyTalonFX(GlobalConstants.ClimberIds.kLeftMotorId);
    public static final TalonFX kRightMotor =
            new LazyTalonFX(GlobalConstants.ClimberIds.kRightMotorId);
    public static final TalonFXInvertType kLeftInvert = TalonFXInvertType.CounterClockwise;
    public static final TalonFXInvertType kRightInvert = TalonFXInvertType.Clockwise;

    public static final Solenoid kPivotPistons =
            new Solenoid(GlobalConstants.kPCMType, GlobalConstants.ClimberIds.kSolenoidPin);

    public static final double kNominalVoltage = 12;
    public static final double kVoltageToHoldRobot = 5;

    public static final double kGearboxReduction = 10 / 40.0 * 24 / 40.0 * 24 / 72.0;
    public static final double kOutputRotationDiameter = Units.inchesToMeters(1.25);
    public static final double kOutputRotationMeters =
            kOutputRotationDiameter * Math.PI * kGearboxReduction;
    public static final double kPosConverstion =
            kOutputRotationMeters / GlobalConstants.kFalconTicksPerRotation;
    public static final double kNativeVelToMpS =
            10 * kOutputRotationMeters / GlobalConstants.kFalconTicksPerRotation;
    public static final double kMaxVelocity = Units.inchesToMeters(10);
    public static final double kMaxAcceleration = Units.inchesToMeters(20);

    public static final double kRevSoftLimit = 0;
    public static final double kFwdSoftLimit = 1000000; // Native units

    public static final double kLengthJustOverLowBar = Units.inchesToMeters(30);
    public static final double kMaxLengthAngled = Units.inchesToMeters(36);
    public static final double kMaxLengthStraight = Units.inchesToMeters(34);
    public static final double kLengthFromStaticHooksToAboveBar = Units.inchesToMeters(10);

    public static final TalonFXConfiguration kLeftConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration kRightConfig = new TalonFXConfiguration();

    static {
        kLeftConfig.slot0.kP = 0.5;
        kLeftConfig.slot0.kI = 0;
        kLeftConfig.slot0.kD = 0;
        kLeftConfig.slot0.kF = 0;

        kLeftConfig.motionCruiseVelocity = kMaxVelocity / kNativeVelToMpS;
        kLeftConfig.motionAcceleration = kMaxAcceleration / kNativeVelToMpS;

        kLeftConfig.forwardSoftLimitEnable = true;
        kLeftConfig.forwardSoftLimitThreshold = kFwdSoftLimit;
        kLeftConfig.reverseSoftLimitEnable = true;
        kLeftConfig.reverseSoftLimitThreshold = kRevSoftLimit;

        kLeftConfig.voltageCompSaturation = kNominalVoltage;

        kLeftMotor.configAllSettings(kLeftConfig, GlobalConstants.kTimeoutMS);
        kLeftMotor.setInverted(kLeftInvert);
    }

    static {
        kRightConfig.slot0.kP = 0.5;
        kRightConfig.slot0.kI = 0;
        kRightConfig.slot0.kD = 0;
        kRightConfig.slot0.kF = 0;

        kRightConfig.forwardSoftLimitEnable = true;
        kRightConfig.forwardSoftLimitThreshold = kFwdSoftLimit;
        kRightConfig.reverseSoftLimitEnable = true;
        kRightConfig.reverseSoftLimitThreshold = kRevSoftLimit;

        kRightConfig.motionCruiseVelocity = kMaxVelocity / kNativeVelToMpS;
        kRightConfig.motionAcceleration = kMaxAcceleration / kNativeVelToMpS;

        kRightConfig.voltageCompSaturation = kNominalVoltage;

        kRightMotor.configAllSettings(kLeftConfig, GlobalConstants.kTimeoutMS);
        kRightMotor.setInverted(kRightInvert);
    }
}
