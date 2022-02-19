package team3647.frc2022.constants;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import team3647.lib.drivers.LazyTalonFX;

public final class DrivetrainConstants {

    public static final TalonFX kLeftMaster =
            new LazyTalonFX(GlobalConstants.DrivetrainIds.kLeftMasterId);
    public static final TalonFX kRightMaster =
            new LazyTalonFX(GlobalConstants.DrivetrainIds.kRightMasterId);
    public static final TalonFX kLeftSlave =
            new LazyTalonFX(GlobalConstants.DrivetrainIds.kLeftSlaveId);
    public static final TalonFX kRightSlave =
            new LazyTalonFX(GlobalConstants.DrivetrainIds.kRightSlaveId);
    public static final PigeonIMU kPigeonIMU = new PigeonIMU(GlobalConstants.DrivetrainIds.kGyroId);

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

    public static final double kS = 0.77918; // Volts
    public static final double kV = 2.4288; // Volts
    public static final double kA = 0.57834; // Volts

    public static final SimpleMotorFeedforward kFeedforward =
            new SimpleMotorFeedforward(kS, kV, kA);

    public static final double kWheelDiameterMeters = 0.1016; // 4inches
    public static final double kTrackWidth = 0;
    public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackWidth);
    public static final double kMaxSpeedMetersPerSecond = 2;
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;

    public static final double kGearboxReduction = 1;

    public static final double kWheelRotationToMeters =
            kWheelDiameterMeters * Math.PI * kGearboxReduction;

    // Multiply by 10 because velocity is in ticks/100ms
    public static final double kFalconVelocityToMpS =
            kWheelRotationToMeters * 10 / GlobalConstants.kFalconTicksPerRotation;

    public static final double kFalconTicksToMeters =
            kWheelRotationToMeters / GlobalConstants.kFalconTicksPerRotation;

    static {
        kLeftMasterConfig.slot0.kP = 2.5812;

        kLeftMasterConfig.supplyCurrLimit.enable = true;
        kLeftMasterConfig.supplyCurrLimit.currentLimit = kStallCurrent;
        kLeftMasterConfig.supplyCurrLimit.triggerThresholdCurrent = kMaxCurrent;
        kLeftMasterConfig.voltageCompSaturation = 12.0;

        kRightMasterConfig.slot0.kP = 0.35;
        kRightMasterConfig.supplyCurrLimit.enable = true;
        kRightMasterConfig.supplyCurrLimit.currentLimit = kStallCurrent;
        kRightMasterConfig.supplyCurrLimit.triggerThresholdCurrent = kMaxCurrent;
        kRightMasterConfig.voltageCompSaturation = 12.0;

        kLeftMaster.configAllSettings(kLeftMasterConfig, GlobalConstants.kTimeoutMS);
        kRightMaster.configAllSettings(kRightMasterConfig, GlobalConstants.kTimeoutMS);
        kLeftSlave.follow(kLeftMaster);
        kRightSlave.follow(kRightMaster);
        kRightMaster.setInverted(TalonFXInvertType.Clockwise);
        kRightSlave.setInverted(InvertType.FollowMaster);
    }

    private DrivetrainConstants() {}
}
