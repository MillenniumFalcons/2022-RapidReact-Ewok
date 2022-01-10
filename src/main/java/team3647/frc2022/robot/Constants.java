package team3647.frc2022.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class CDrivetrain {
        public static final int kLeftMasterID = 1;
        public static final int kLeftSlaveID = 2;
        public static final int kRightMasterID = 3;
        public static final int kRightSlaveID = 4;

        public static final int kPigeonID = 5;

        public static final TalonFX kLeftMaster = new TalonFX(kLeftMasterID);
        public static final TalonFX kRightMaster = new TalonFX(kRightMasterID);
        public static final TalonFX kLeftSlave = new TalonFX(kLeftSlaveID);
        public static final TalonFX kRightSlave = new TalonFX(kRightSlaveID);
        public static final PigeonIMU kPigeonIMU = new PigeonIMU(kPigeonID);

        public static final double kStallCurrent = 0;
        public static final double kMaxCurrent = 0;

        public static final TalonFXConfiguration kLeftMasterConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration kRightMasterConfig = new TalonFXConfiguration();

        public static final double kS = 0; // Volts
        public static final double kV = 0; // Volts
        public static final double kA = 0; // Volts

        public static final SimpleMotorFeedforward kFeedforward =
                new SimpleMotorFeedforward(kS, kV, kA);

        public static final double kWheelDiameterMeters = 0.1524;
        public static final double kTrackWidth = 0;
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;

        public static final double kFalconTicksPerRotation = 2048;
        public static final double kGearboxReduction = 1;

        public static final double kWheelRotationToMeters =
                kWheelDiameterMeters * Math.PI * kGearboxReduction;

        // Multiply by 10 because velocity is in ticks/100ms
        public static final double kFalconVelocityToMpS =
                kWheelRotationToMeters * 10 / kFalconTicksPerRotation;

        public static final double kFalconTicksToMeters =
                kWheelRotationToMeters / kFalconTicksPerRotation;

        static {
            kLeftMasterConfig.slot0.kP = 0.35;

            kLeftMasterConfig.supplyCurrLimit.enable = true;
            kLeftMasterConfig.supplyCurrLimit.currentLimit = kStallCurrent;
            kLeftMasterConfig.supplyCurrLimit.triggerThresholdCurrent = kMaxCurrent;
            kLeftMasterConfig.voltageCompSaturation = 12.0;

            kRightMasterConfig.slot0.kP = 0.35;
            kRightMasterConfig.supplyCurrLimit.enable = true;
            kRightMasterConfig.supplyCurrLimit.currentLimit = kStallCurrent;
            kRightMasterConfig.supplyCurrLimit.triggerThresholdCurrent = kMaxCurrent;
            kRightMasterConfig.voltageCompSaturation = 12.0;

            kLeftMaster.configAllSettings(kLeftMasterConfig);
            kRightMaster.configAllSettings(kRightMasterConfig);
            kLeftSlave.follow(kLeftMaster);
            kRightSlave.follow(kRightMaster);
            kRightMaster.setInverted(InvertType.InvertMotorOutput);
            kRightSlave.setInverted(InvertType.FollowMaster);
        }

        private CDrivetrain() {}
    }

    private Constants() {}
}
