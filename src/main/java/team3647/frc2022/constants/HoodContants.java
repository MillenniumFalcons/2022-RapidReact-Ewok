package team3647.frc2022.constants;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.DigitalInput;

public class HoodContants {

    public static final TalonFX kHoodMotor = new TalonFX(GlobalConstants.HoodIds.kMotorId);
    public static final CANCoder kEncoder = new CANCoder(GlobalConstants.HoodIds.kEncoderId);
    public static final DigitalInput kZeroSensor =
            new DigitalInput(GlobalConstants.HoodIds.kBannerSensorPin);

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
        kMasterConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
        kMasterConfig.remoteFilter0.remoteSensorDeviceID = GlobalConstants.HoodIds.kEncoderId;
        kMasterConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;

        kHoodMotor.configAllSettings(kMasterConfig);
    }
}
