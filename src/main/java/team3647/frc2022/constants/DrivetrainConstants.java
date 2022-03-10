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

    public static final double kNominalVoltage = 10;
    public static final double kStallCurrent = 35;
    public static final double kMaxCurrent = 60;

    public static final TalonFXConfiguration kLeftMasterConfig = new TalonFXConfiguration();
    public static final TalonFXConfiguration kRightMasterConfig = new TalonFXConfiguration();

    public static final double kS = 0.70649; // Volts
    public static final double kV = 2.1964; // Volts
    public static final double kA = 0.33557; // Volts

    public static final SimpleMotorFeedforward kFeedforward =
            new SimpleMotorFeedforward(kS, kV, kA);

    public static final double kWheelDiameterMeters = 0.1016; // 4inches
    public static final double kTrackWidth = 0.75;
    public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackWidth);
    public static final double kMaxSpeedMetersPerSecondSlow = 2;
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.5;

    public static final double kGearboxReduction = 10 / 42.0 * 24 / 40.;

    public static final double kWheelRotationToMeters =
            kWheelDiameterMeters * Math.PI * kGearboxReduction;

    // Multiply by 10 because velocity is in ticks/100ms
    public static final double kFalconVelocityToMpS =
            kWheelRotationToMeters * 10 / GlobalConstants.kFalconTicksPerRotation;

    public static final double kFalconTicksToMeters =
            kWheelRotationToMeters / GlobalConstants.kFalconTicksPerRotation;

    static {
        kLeftMasterConfig.slot0.kP = 1;
        kLeftMasterConfig.slot0.kI = 0;
        kLeftMasterConfig.slot0.kD = 0;

        kLeftMasterConfig.supplyCurrLimit.enable = true;
        kLeftMasterConfig.supplyCurrLimit.currentLimit = kStallCurrent;
        kLeftMasterConfig.supplyCurrLimit.triggerThresholdCurrent = kMaxCurrent;
        kLeftMasterConfig.voltageCompSaturation = kNominalVoltage;

        kLeftMasterConfig.slot0.kP = 0.000001;
        kRightMasterConfig.slot0.kI = 0;
        kRightMasterConfig.slot0.kD = 0;

        kRightMasterConfig.supplyCurrLimit.enable = true;
        kRightMasterConfig.supplyCurrLimit.currentLimit = kStallCurrent;
        kRightMasterConfig.supplyCurrLimit.triggerThresholdCurrent = kMaxCurrent;
        kRightMasterConfig.voltageCompSaturation = kNominalVoltage;

        kLeftMaster.configAllSettings(kLeftMasterConfig, GlobalConstants.kTimeoutMS);
        kRightMaster.configAllSettings(kRightMasterConfig, GlobalConstants.kTimeoutMS);
        kLeftSlave.follow(kLeftMaster);
        kRightSlave.follow(kRightMaster);
        kRightMaster.setInverted(TalonFXInvertType.Clockwise);
        kRightSlave.setInverted(InvertType.FollowMaster);
        kLeftMaster.enableVoltageCompensation(true);
        kLeftSlave.enableVoltageCompensation(true);
        kRightMaster.enableVoltageCompensation(true);
        kRightSlave.enableVoltageCompensation(true);
    }

    private DrivetrainConstants() {}
}
