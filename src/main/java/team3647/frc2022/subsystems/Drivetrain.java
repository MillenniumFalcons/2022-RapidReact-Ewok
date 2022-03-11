package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.wpi.HALMethods;

public final class Drivetrain implements PeriodicSubsystem {

    private final TalonFX leftMaster;
    private final TalonFX leftSlave;
    private final TalonFX rightMaster;
    private final TalonFX rightSlave;

    private final PigeonIMU pigeonIMU;

    private final SimpleMotorFeedforward feedforward;
    // private final DifferentialDrivePoseEstimator poseEstimator;
    private final DifferentialDriveOdometry odometry;
    private PeriodicIO periodicIO = new PeriodicIO();

    private final double velocityConversion;
    private final double displacementConversion;

    private final double nominalVoltage;
    private final double kDt;

    public Drivetrain(
            TalonFX leftMaster,
            TalonFX rightMaster,
            TalonFX leftSlave,
            TalonFX rightSlave,
            PigeonIMU pigeonIMU,
            SimpleMotorFeedforward feedforward,
            DifferentialDrivePoseEstimator poseEstimator,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt) {
        this.leftMaster = leftMaster;
        this.leftSlave = leftSlave;
        this.rightMaster = rightMaster;
        this.rightSlave = rightSlave;
        this.pigeonIMU = pigeonIMU;
        this.feedforward = feedforward;
        this.velocityConversion = velocityConversion;
        this.displacementConversion = positionConversion;
        // this.poseEstimator = poseEstimator;
        this.odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(periodicIO.heading));
        this.nominalVoltage = nominalVoltage;
        this.kDt = kDt;
        setStatusFramesThatDontMatter(rightMaster, 255);
        setStatusFramesThatDontMatter(leftMaster, 255);
        setStatusFrames(leftSlave, 255);
        setStatusFrames(rightSlave, 255);
        setPidgeonStatusFrames(255);
    }

    public static class PeriodicIO {
        // inputs
        public double timestamp = 0;
        /** Meters per second */
        public DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds();

        public Pose2d pose = new Pose2d();
        public double ypr[] = new double[] {0, 0, 0};

        /** Meters */
        public double leftPosition = 0;
        /** Meters */
        public double rightPosition = 0;
        /** Degrees -180 to 180 */
        public double heading = 0;

        // outputs
        public ControlMode controlMode = ControlMode.Disabled;

        public double leftOutput = 0;
        public double rightOutput = 0;

        /** Volts */
        public double leftFeedForward = 0;
        /** Volts */
        public double rightFeedForward = 0;

        public double rightMasterVelocity = 0;
        public double leftMasterVelocity = 0;
    }

    @Override
    public void init() {
        setToBrake();
        resetEncoders();
        resetOdometry();
    }

    @Override
    public synchronized void readPeriodicInputs() {
        periodicIO.leftPosition = leftMaster.getSelectedSensorPosition() * displacementConversion;
        periodicIO.rightPosition = rightMaster.getSelectedSensorPosition() * displacementConversion;

        periodicIO.wheelSpeeds.leftMetersPerSecond =
                leftMaster.getSelectedSensorVelocity() * velocityConversion;
        periodicIO.wheelSpeeds.rightMetersPerSecond =
                rightMaster.getSelectedSensorVelocity() * velocityConversion;
        periodicIO.leftMasterVelocity = periodicIO.wheelSpeeds.leftMetersPerSecond;
        periodicIO.rightMasterVelocity = periodicIO.wheelSpeeds.rightMetersPerSecond;

        pigeonIMU.getYawPitchRoll(periodicIO.ypr);
        periodicIO.heading = Math.IEEEremainder(periodicIO.ypr[0], 360);
        SmartDashboard.putNumber("Gyro reading", periodicIO.ypr[0]);

        periodicIO.timestamp = Timer.getFPGATimestamp();
        periodicIO.pose =
                odometry.update(
                        Rotation2d.fromDegrees(periodicIO.heading),
                        periodicIO.leftPosition,
                        periodicIO.rightPosition);
        /*periodicIO.pose =
        poseEstimator.update(
                Rotation2d.fromDegrees(periodicIO.heading),
                periodicIO.wheelSpeeds,
                periodicIO.leftPosition,
                periodicIO.rightPosition);*/

    }

    @Override
    public void writePeriodicOutputs() {
        if (leftMaster.hasResetOccurred()) {
            setStatusFramesThatDontMatter(leftMaster, 255);
        }
        if (rightMaster.hasResetOccurred()) {
            setStatusFramesThatDontMatter(rightMaster, 255);
        }

        if (leftSlave.hasResetOccurred()) {
            setStatusFrames(leftSlave, 255);
        }
        if (rightSlave.hasResetOccurred()) {
            setStatusFrames(rightSlave, 255);
        }

        if (pigeonIMU.hasResetOccurred()) {
            setPidgeonStatusFrames(255);
        }

        leftMaster.set(
                periodicIO.controlMode,
                periodicIO.leftOutput,
                DemandType.ArbitraryFeedForward,
                periodicIO.leftFeedForward / nominalVoltage);
        rightMaster.set(
                periodicIO.controlMode,
                periodicIO.rightOutput,
                DemandType.ArbitraryFeedForward,
                periodicIO.rightFeedForward / nominalVoltage);
    }

    public boolean getRightMasterInvert() {
        return rightMaster.getInverted();
    }

    public boolean getRightSlaveInvert() {
        return rightSlave.getInverted();
    }

    @Override
    public void periodic() {
        // readPeriodicInputs(); Write in robot addPeriodic
        writePeriodicOutputs();
    }

    @Override
    public void end() {
        setOpenloop(0, 0);
    }

    public void setVelocityLeftRight(double leftVelocity, double rightVelocity) {
        DifferentialDriveWheelSpeeds driveLeftRight =
                new DifferentialDriveWheelSpeeds(leftVelocity, rightVelocity);
        setVelocity(driveLeftRight);
    }

    public void setVelocity(DifferentialDriveWheelSpeeds wheelSpeeds) {
        if (wheelSpeeds == null) {
            HALMethods.sendDSError(
                    "wheelSpeeds in setVelocity(DifferentialDriveWheelSpeeds) was null");
            end();
            return;
        }
        SmartDashboard.putNumber(
                "Right: Actual - Requested", getRightVelocity() - wheelSpeeds.rightMetersPerSecond);
        SmartDashboard.putNumber(
                "Left: Actual - Requested", getLeftVelocity() - wheelSpeeds.leftMetersPerSecond);
        periodicIO.controlMode = ControlMode.Velocity;
        periodicIO.leftFeedForward = feedforward.calculate(wheelSpeeds.leftMetersPerSecond);
        periodicIO.rightFeedForward = feedforward.calculate(wheelSpeeds.rightMetersPerSecond);
        periodicIO.leftOutput = wheelSpeeds.leftMetersPerSecond / velocityConversion;
        periodicIO.rightOutput = wheelSpeeds.rightMetersPerSecond / velocityConversion;
    }

    public void setOpenloop(double leftOut, double rightOut) {
        periodicIO.controlMode = ControlMode.PercentOutput;
        periodicIO.leftFeedForward = 0;
        periodicIO.rightFeedForward = 0;
        periodicIO.leftOutput = leftOut;
        periodicIO.rightOutput = rightOut;
    }

    public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
        WheelSpeeds ws = DifferentialDrive.curvatureDriveIK(xSpeed, zRotation, isQuickTurn);
        // isQuickTurn);
        setOpenloop(ws.left, ws.right);
    }

    public void setToCoast() {
        setNeutralMode(NeutralMode.Coast);
    }

    public void setToBrake() {
        setNeutralMode(NeutralMode.Brake);
    }

    public void setNeutralMode(NeutralMode mode) {
        leftMaster.setNeutralMode(mode);
        rightMaster.setNeutralMode(mode);
        leftSlave.setNeutralMode(mode);
        rightSlave.setNeutralMode(mode);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return periodicIO.wheelSpeeds;
    }

    public Pose2d getPose() {
        return periodicIO.pose;
    }

    public double getTimestamp() {
        return periodicIO.timestamp;
    }

    public double getDrivetrainXMeters() {
        return periodicIO.pose.getX();
    }

    public double getDrivetrainYMeters() {
        return periodicIO.pose.getY();
    }

    public double getLeftMasterVelocity() {
        return periodicIO.leftMasterVelocity;
    }

    public double getRightMasterVelocity() {
        return periodicIO.rightMasterVelocity;
    }

    public double getLeftVelocity() {
        return periodicIO.wheelSpeeds.leftMetersPerSecond;
    }

    public double getRightVelocity() {
        return periodicIO.wheelSpeeds.rightMetersPerSecond;
    }

    public void resetOdometry() {
        setOdometry(new Pose2d(), new Rotation2d());
    }

    public void setOdometry(Pose2d pose, Rotation2d angle) {
        resetEncoders();
        pigeonIMU.setYaw(angle.getDegrees());
        periodicIO = new PeriodicIO();
        odometry.resetPosition(pose, angle);
    }

    private void resetEncoders() {
        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);
    }

    public boolean isStopped(double threshold) {
        return Math.abs(getRightVelocity()) < threshold && Math.abs(getLeftVelocity()) < threshold;
    }

    public boolean isStopped() {
        return isStopped(0.1);
    }

    @Override
    public String getName() {
        return "Drivetrain";
    }

    private void setStatusFrames(TalonFX device, int timeout) {
        device.setStatusFramePeriod(StatusFrame.Status_1_General, timeout);
        device.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, timeout);
        device.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, timeout);
        device.setStatusFramePeriod(StatusFrame.Status_6_Misc, timeout);
        device.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, timeout);
        device.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, timeout);
        device.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, timeout);
        device.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, timeout);
        device.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, timeout, timeout);
        device.setStatusFramePeriod(StatusFrame.Status_17_Targets1, timeout);
    }

    private void setStatusFramesThatDontMatter(TalonFX device, int timeout) {
        device.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, timeout);
        device.setStatusFramePeriod(StatusFrame.Status_6_Misc, timeout);
        device.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, timeout);
        device.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, timeout);
        device.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, timeout);
        device.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, timeout);
        device.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, timeout, timeout);
        device.setStatusFramePeriod(StatusFrame.Status_17_Targets1, timeout);
    }

    private void setPidgeonStatusFrames(int frameMS) {
        // pigeonIMU.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, frameMS);
        pigeonIMU.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_2_GeneralCompass, frameMS);
        pigeonIMU.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, frameMS);
        pigeonIMU.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, frameMS);
        pigeonIMU.setStatusFramePeriod(PigeonIMU_StatusFrame.RawStatus_4_Mag, frameMS);
        pigeonIMU.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, frameMS);
        pigeonIMU.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_4_Mag, frameMS);
        pigeonIMU.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, frameMS);
    }
}
