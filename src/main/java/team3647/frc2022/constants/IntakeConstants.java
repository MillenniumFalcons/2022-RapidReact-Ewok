package team3647.frc2022.constants;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Solenoid;
import team3647.lib.drivers.LazyTalonFX;

public final class IntakeConstants {

    public static final InvertType kMasterInverted = InvertType.None;
    public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final SimpleMotorFeedforward kFeedForward =
            new SimpleMotorFeedforward(kS, kV, kA);
    public static final double kNominalVoltage = 12;

    public static final TalonFX kIntakeMotor = new LazyTalonFX(GlobalConstants.IntakeIds.kMotorId);
    public static final Solenoid kPistons =
            new Solenoid(GlobalConstants.kPCMType, GlobalConstants.IntakeIds.kSolenoidPin);

    public static final double kWheelDiameterMeters = 0.0508;
    public static final double nativeVelToSurfaceMpS =
            10 * kWheelDiameterMeters / GlobalConstants.kFalconTicksPerRotation;

    static {
        kMasterConfig.slot0.kP = 0;
        kMasterConfig.slot0.kI = 0;
        kMasterConfig.slot0.kD = 0;
        kMasterConfig.slot0.kF = 0;

        kMasterConfig.voltageCompSaturation = 12;

        kIntakeMotor.configAllSettings(kMasterConfig, GlobalConstants.kTimeoutMS);
    }

    private IntakeConstants() {}
}
