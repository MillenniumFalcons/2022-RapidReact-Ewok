package team3647.frc2022.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double kFalconTicksPerRotation = 2048;
    public static final PneumaticsModuleType kPCMType = PneumaticsModuleType.CTREPCM;

    public static final class CDrivetrain {
        public static final int kLeftMasterID = 1;
        public static final int kLeftSlaveID = 2;
        public static final int kRightMasterID = 3;
        public static final int kRightSlaveID = 4;

        public static final int kPigeonID = 16;

        public static final TalonFX kLeftMaster = new TalonFX(kLeftMasterID);
        public static final TalonFX kRightMaster = new TalonFX(kRightMasterID);
        public static final TalonFX kLeftSlave = new TalonFX(kLeftSlaveID);
        public static final TalonFX kRightSlave = new TalonFX(kRightSlaveID);
        public static final PigeonIMU kPigeonIMU = new PigeonIMU(kPigeonID);

        // increasing the standard deviation makes estimator trusts the data less
        // need to figure out how to use this and tune the stdDevs
        public static final DifferentialDrivePoseEstimator kPoseEstimator =
                new DifferentialDrivePoseEstimator(
                        new Rotation2d(),
                        new Pose2d(),
                        new MatBuilder<>(Nat.N5(), Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02),
                        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01),
                        new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01));

        public static final double kNominalVoltage = 12;
        public static final double kStallCurrent = 35;
        public static final double kMaxCurrent = 60;

        public static final TalonFXConfiguration kLeftMasterConfig = new TalonFXConfiguration();
        public static final TalonFXConfiguration kRightMasterConfig = new TalonFXConfiguration();

        public static final double kS = 0; // Volts
        public static final double kV = 0; // Volts
        public static final double kA = 0; // Volts

        public static final SimpleMotorFeedforward kFeedforward =
                new SimpleMotorFeedforward(kS, kV, kA);

        public static final double kWheelDiameterMeters = 0.1016; // 4inches
        public static final double kTrackWidth = 0;
        public static final double kMaxSpeedMetersPerSecond = 2;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;

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

    public static final class CIntake {
        public static final int kMasterID = 5; // CHECK!
        public static final int kSolenoidPin = 0;
        public static final InvertType kMasterInverted = InvertType.None;
        public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

        public static final double kWheelDiameterMeters = 0.0508;

        public static final double nativeVelToSurfaceMpS =
                10 * kWheelDiameterMeters / kFalconTicksPerRotation;
        public static final TalonFX kIntakeMotor = new TalonFX(kMasterID);
        public static final Solenoid kPistons = new Solenoid(kPCMType, kSolenoidPin);

        static {
            kMasterConfig.slot0.kP = 0;
            kMasterConfig.slot0.kI = 0;
            kMasterConfig.slot0.kD = 0;
            kMasterConfig.slot0.kF = 0;

            kMasterConfig.voltageCompSaturation = 12;

            kIntakeMotor.configAllSettings(kMasterConfig);
        }
    }

    public static final class CHood {
        public static final int kMotorID = 7;
        public static final int kCANCoderID = 12;
        public static final TalonFXConfiguration kMasterConfig = new TalonFXConfiguration();

        public static final TalonFX kHoodMotor = new TalonFX(kMotorID);

        static {
            kMasterConfig.slot0.kP = 0;
            kMasterConfig.slot0.kI = 0;
            kMasterConfig.slot0.kD = 0;
            kMasterConfig.slot0.kF = 0;

            kMasterConfig.voltageCompSaturation = 12;
            kMasterConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
            kMasterConfig.remoteFilter0.remoteSensorDeviceID = kCANCoderID;
            kMasterConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;

            kHoodMotor.configAllSettings(kMasterConfig);
        }
    }

    private Constants() {}
}
