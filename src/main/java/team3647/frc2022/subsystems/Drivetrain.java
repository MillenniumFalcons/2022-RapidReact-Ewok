package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.wpi.HALMethods;

public final class Drivetrain implements PeriodicSubsystem {

    private final TalonFX leftMaster;
    private final TalonFX leftSlave;
    private final TalonFX rightMaster;
    private final TalonFX rightSlave;

    private final PigeonIMU pigeonIMU;

    private final SimpleMotorFeedforward feedforward;
    private final DifferentialDrivePoseEstimator poseEstimator;
    private PeriodicIO periodicIO = new PeriodicIO();

    private final double velocityConversion;
    private final double displacementConversion;

    private final double nominalVoltage;

    public static final double kDefaultQuickStopThreshold = 0.2;
    public static final double kDefaultQuickStopAlpha = 0.1;
    private double m_quickStopThreshold = kDefaultQuickStopThreshold;
    private double m_quickStopAlpha = kDefaultQuickStopAlpha;
    private double m_quickStopAccumulator;

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
            double nominalVoltage) {
        this.leftMaster = leftMaster;
        this.leftSlave = leftSlave;
        this.rightMaster = rightMaster;
        this.rightSlave = rightSlave;
        this.pigeonIMU = pigeonIMU;
        this.feedforward = feedforward;
        this.velocityConversion = velocityConversion;
        this.displacementConversion = positionConversion;
        this.poseEstimator = poseEstimator;
        this.nominalVoltage = nominalVoltage;
    }

    public static class PeriodicIO {
        // inputs
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
    }

    @Override
    public void init() {
        setToBrake();
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.leftPosition = leftMaster.getSelectedSensorPosition() * displacementConversion;
        periodicIO.rightPosition = rightMaster.getSelectedSensorPosition() * displacementConversion;

        periodicIO.wheelSpeeds.leftMetersPerSecond =
                leftMaster.getSelectedSensorVelocity() * velocityConversion;
        periodicIO.wheelSpeeds.rightMetersPerSecond =
                rightMaster.getSelectedSensorVelocity() * velocityConversion;

        pigeonIMU.getYawPitchRoll(periodicIO.ypr);
        periodicIO.heading = -Math.IEEEremainder(periodicIO.ypr[0], 360);

        periodicIO.pose =
                poseEstimator.update(
                        Rotation2d.fromDegrees(periodicIO.heading),
                        periodicIO.wheelSpeeds,
                        periodicIO.leftPosition,
                        periodicIO.rightPosition);
    }

    @Override
    public void writePeriodicOutputs() {
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

    @Override
    public void periodic() {
        readPeriodicInputs();
        writePeriodicOutputs();
    }

    @Override
    public void end() {
        setOpenloop(0, 0);
    }

    public void setVelocity(DifferentialDriveWheelSpeeds wheelSpeeds) {
        if (wheelSpeeds == null) {
            HALMethods.sendDSError(
                    "wheelSpeeds in setVelocity(DifferentialDriveWheelSpeeds) was null");
            end();
            return;
        }
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
        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        xSpeed = applyDeadband(xSpeed, .09);

        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
        zRotation = applyDeadband(zRotation, .09);

        double angularPower;
        boolean overPower;

        if (isQuickTurn) {
            if (Math.abs(xSpeed) < m_quickStopThreshold) {
                m_quickStopAccumulator =
                        (1 - m_quickStopAlpha) * m_quickStopAccumulator
                                + m_quickStopAlpha * MathUtil.clamp(zRotation, -1.0, 1.0) * 2;
            }
            overPower = true;
            angularPower = zRotation;
        } else {
            overPower = false;
            angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

            if (m_quickStopAccumulator > 1) {
                m_quickStopAccumulator -= 1;
            } else if (m_quickStopAccumulator < -1) {
                m_quickStopAccumulator += 1;
            } else {
                m_quickStopAccumulator = 0.0;
            }
        }

        double leftMotorOutput = xSpeed + angularPower;
        double rightMotorOutput = xSpeed - angularPower;

        // If rotation is overpowered, reduce both outputs to within acceptable range
        if (overPower) {
            if (leftMotorOutput > 1.0) {
                rightMotorOutput -= leftMotorOutput - 1.0;
                leftMotorOutput = 1.0;
            } else if (rightMotorOutput > 1.0) {
                leftMotorOutput -= rightMotorOutput - 1.0;
                rightMotorOutput = 1.0;
            } else if (leftMotorOutput < -1.0) {
                rightMotorOutput -= leftMotorOutput + 1.0;
                leftMotorOutput = -1.0;
            } else if (rightMotorOutput < -1.0) {
                leftMotorOutput -= rightMotorOutput + 1.0;
                rightMotorOutput = -1.0;
            }
        }

        // Normalize the wheel speeds
        double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
        if (maxMagnitude > 1.0) {
            leftMotorOutput /= maxMagnitude;
            rightMotorOutput /= maxMagnitude;
        }

        setOpenloop(leftMotorOutput, rightMotorOutput);
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

    public void resetOdometry() {
        setOdometry(new Pose2d(), new Rotation2d());
    }

    public void setOdometry(Pose2d pose, Rotation2d angle) {
        resetEncoders();
        pigeonIMU.setYaw(angle.getDegrees());
        periodicIO = new PeriodicIO();
        poseEstimator.resetPosition(pose, angle);
    }

    private void resetEncoders() {
        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);
    }

    /**
     * Returns 0.0 if the given value is within the specified range around zero. The remaining range
     * between the deadband and 1.0 is scaled from 0.0 to 1.0.
     *
     * @param value value to clip
     * @param deadband range around zero
     */
    private double applyDeadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    @Override
    public String getName() {
        return "Drivetrain";
    }
}
