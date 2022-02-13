package team3647.lib;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Timer;
import java.util.LinkedList;
import java.util.List;

public abstract class TalonFXSubsystem implements PeriodicSubsystem {

    private final TalonFX master;
    private final List<TalonFX> followers = new LinkedList<>();
    private final double positionConversion;
    private final double velocityConversion;
    private final double nominalVoltage;
    protected final double kDt;
    public static final int kLongStatusTimeMS = 255;
    public static final int kTimeoutMS = 100;

    private PeriodicIO periodicIO = new PeriodicIO();

    protected TalonFXSubsystem(
            TalonFX master,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt) {
        this.master = master;
        this.velocityConversion = velocityConversion;
        this.positionConversion = positionConversion;
        this.nominalVoltage = nominalVoltage;
        this.kDt = kDt;
        this.master.clearStickyFaults(kLongStatusTimeMS);
    }

    public static class PeriodicIO {
        // Inputs
        public double position = 0;
        public double velocity = 0;
        public double current = 0;
        public double timestamp = 0;
        public double masterCurrent = 0;
        public double nativePosition = 0;

        // Outputs
        public ControlMode controlMode = ControlMode.Disabled;
        public double demand = 0;
        public double feedforward = 0;
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.nativePosition = master.getSelectedSensorPosition();
        periodicIO.position = periodicIO.nativePosition * positionConversion;
        periodicIO.velocity = master.getSelectedSensorVelocity() * velocityConversion;
        periodicIO.current = master.getStatorCurrent();
        periodicIO.timestamp = Timer.getFPGATimestamp();
        periodicIO.masterCurrent = master.getSupplyCurrent();
    }

    @Override
    public void writePeriodicOutputs() {
        master.set(
                periodicIO.controlMode,
                periodicIO.demand,
                DemandType.ArbitraryFeedForward,
                periodicIO.feedforward / nominalVoltage);

        for (var follower : followers) {
            if (follower.hasResetOccurred()) {
                setStatusFrames(follower, kLongStatusTimeMS);
            }
        }
    }

    @Override
    public void end() {
        setOpenloop(0);
    }

    public void setOpenloop(double output) {
        periodicIO.controlMode = ControlMode.PercentOutput;
        periodicIO.demand = output;
        periodicIO.feedforward = 0;
    }

    /**
     * Raw PID (not motion magic)
     *
     * @param position in units of positionConvertsion (degrees/meters/etc..)
     * @param feedforward in volts
     */
    protected void setPosition(double position, double feedforward) {
        periodicIO.controlMode = ControlMode.Position;
        periodicIO.feedforward = feedforward;
        periodicIO.demand = position / positionConversion;
    }

    /**
     * Motion Magic position
     *
     * @param position in units of positionConvertsion (degrees/meters/etc..)
     * @param feedforward in volts
     */
    protected void setPositionMotionMagic(double position, double feedforward) {
        periodicIO.controlMode = ControlMode.MotionMagic;
        periodicIO.feedforward = feedforward;
        periodicIO.demand = position / positionConversion;
    }

    /**
     * @param velocity in the units of velocityConversion (RPM?, Surface speed?)
     * @param feedforward in volts
     */
    protected void setVelocity(double velocity, double feedforward) {
        periodicIO.controlMode = ControlMode.Velocity;
        periodicIO.feedforward = feedforward;
        periodicIO.demand = velocity / velocityConversion;
    }

    /** Sets all motors to brake mode */
    public void setToBrake() {
        setNeutralMode(NeutralMode.Brake);
    }

    /** Sets all motors to coast mode */
    public void setToCoast() {
        setNeutralMode(NeutralMode.Coast);
    }

    /**
     * Sets master and all followers to the mode
     *
     * @param mode either Brake or Coast
     */
    public void setNeutralMode(NeutralMode mode) {
        master.setNeutralMode(mode);
        for (TalonFX follower : followers) {
            follower.setNeutralMode(mode);
        }
    }

    /** Sets the selected sensor to 0 (default) */
    public void resetEncoder() {
        setEncoder(0);
    }

    /**
     * sets the selected sensor to position
     *
     * @param position position in output units
     */
    public void setEncoder(double position) {
        master.setSelectedSensorPosition(position / positionConversion);
    }

    /** @return the velocity in the output units */
    public double getVelocity() {
        return periodicIO.velocity;
    }

    /** @return ths position in the output units */
    public double getPosition() {
        return periodicIO.position;
    }

    /** @return the timestamp for the position and velocity measurements */
    public double getTimestamp() {
        return periodicIO.timestamp;
    }

    public double getMasterCurrent() {
        return periodicIO.masterCurrent;
    }

    public double getNativePos() {
        return periodicIO.nativePosition;
    }

    protected void addFollower(TalonFX follower, FollowerType followerType, InvertType invertType) {
        follower.follow(master, followerType);
        follower.setInverted(invertType);
        setStatusFrames(follower, kLongStatusTimeMS);

        followers.add(follower);
    }

    protected void setStatusFrames(TalonFX device, int timeout) {
        device.setStatusFramePeriod(StatusFrame.Status_1_General, timeout, kTimeoutMS);
        device.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, timeout, kTimeoutMS);
        device.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, timeout, kTimeoutMS);
        device.setStatusFramePeriod(StatusFrame.Status_6_Misc, timeout, kTimeoutMS);
        device.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, timeout, kTimeoutMS);
        device.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, timeout, kTimeoutMS);
        device.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, timeout, kTimeoutMS);
        device.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, timeout, kTimeoutMS);
        device.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, timeout, kTimeoutMS);
        device.setStatusFramePeriod(StatusFrame.Status_17_Targets1, timeout, kTimeoutMS);
    }

    protected void setStatusFramesThatDontMatter(TalonFX device, int timeout) {
        device.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, timeout, kTimeoutMS);
        device.setStatusFramePeriod(StatusFrame.Status_6_Misc, timeout, kTimeoutMS);
        device.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, timeout, kTimeoutMS);
        device.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, timeout, kTimeoutMS);
        device.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, timeout, kTimeoutMS);
        device.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, timeout, kTimeoutMS);
        device.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, timeout, kTimeoutMS);
        device.setStatusFramePeriod(StatusFrame.Status_17_Targets1, timeout, kTimeoutMS);
    }
}
