package team3647.frc2022.constants;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class GlobalConstants {
    public static final double kFalconTicksPerRotation = 2048;
    public static final PneumaticsModuleType kPCMType = PneumaticsModuleType.CTREPCM;
    public static final double kDt = 0.02; // 20ms loop
    public static final int kTimeoutMS = 255;
    public static double kCenterOffsetMeters = Units.inchesToMeters(34);
    public static double kDistanceTarmacToGoalCenter = Units.inchesToMeters(84);
    public static double kDistanceFarToGoalCenter = Units.feetToMeters(14) + kCenterOffsetMeters;

    public static final class DrivetrainIds {
        public static final int kLeftMasterId = 1;
        public static final int kRightMasterId = 3;
        public static final int kLeftSlaveId = 2;
        public static final int kRightSlaveId = 4;
        public static final int kGyroId = 16;
    }

    public static final class IntakeIds {
        public static final int kIntakeMotorId = 5;
        public static final int kDeployMotorId = 15;
        public static final int kSolenoidPin = 0;
    }

    public static final class ColumnBottomIds {
        public static final int kMotorId = 6;
        public static final int kSolenoidPin = 2;
    }

    public static final class ColumnTopIds {
        public static final int kMotorId = 7;
        public static final int kTopSensorPin = 0;
    }

    public static final class VerticalRollersIds {
        public static final int kMotorId = 8;
    }

    public static final class TurretIds {
        public static final int kMotorId = 9;
    }

    public static final class HoodIds {
        public static final int kMotorId = 10;
    }

    public static final class FlywheelIds {
        public static final int kMasterId = 11;
        public static final int kFollowerId = 12;
    }

    public static final class ClimberIds {
        public static final int kLeftMotorId = 13;
        public static final int kRightMotorId = 14;
        public static final int kSolenoidPin = 1;
    }
}
