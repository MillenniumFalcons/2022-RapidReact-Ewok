package team3647.lib;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import java.util.LinkedList;
import java.util.List;

public abstract class TalonFXSubsystem implements PeriodicSubsystem {

  private final TalonFX master;
  private final List<TalonFX> followers = new LinkedList<>();
  private final double positionConversion;
  private final double velocityConversion;

  private PeriodicIO periodicIO = new PeriodicIO();

  protected TalonFXSubsystem(TalonFX master, double velocityConversion, double positionConversion) {
    this.master = master;
    this.velocityConversion = velocityConversion;
    this.positionConversion = positionConversion;
  }

  public static class PeriodicIO {
    // Inputs
    public double position = 0;
    public double velocity = 0;
    public double current = 0;

    // Outputs
    public ControlMode controlMode = ControlMode.Disabled;
    public double demand = 0;
    public double feedforward = 0;
  }

  @Override
  public void readPeriodicInputs() {
    periodicIO.position = master.getSelectedSensorVelocity() * velocityConversion;
    periodicIO.velocity = master.getSelectedSensorVelocity() * positionConversion;
    periodicIO.current = master.getStatorCurrent();
  }

  @Override
  public void writePeriodicOutputs() {
    master.set(
        periodicIO.controlMode,
        periodicIO.demand,
        DemandType.ArbitraryFeedForward,
        periodicIO.feedforward);
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
   * @param position in SI units (degrees/meters/etc..)
   * @param feedforward in [-1,1]
   */
  protected void setPosition(double position, double feedforward) {
    periodicIO.controlMode = ControlMode.Position;
    periodicIO.feedforward = feedforward;
    periodicIO.demand = position / positionConversion;
  }

  /**
   * Motion Magic position
   *
   * @param position in SI units (degrees/meters/etc..)
   * @param feedforward in [-1,1]
   */
  protected void setPositionMotionMagic(double position, double feedforward) {
    periodicIO.controlMode = ControlMode.MotionMagic;
    periodicIO.feedforward = feedforward;
    periodicIO.demand = position / positionConversion;
  }

  /**
   * @param velocity in the units of velocityConversion (RPM?, Surface speed?)
   * @param feedforward in [-1,1]
   */
  protected void setVelocity(double velocity, double feedforward) {
    periodicIO.controlMode = ControlMode.Velocity;
    periodicIO.feedforward = feedforward;
    periodicIO.demand = velocity / velocityConversion;
  }

  public void setToBrake() {
    setNeutralMode(NeutralMode.Brake);
  }

  public void setToCoast() {
    setNeutralMode(NeutralMode.Coast);
  }

  public void setNeutralMode(NeutralMode mode) {
    master.setNeutralMode(mode);
    for (TalonFX follower : followers) {
      follower.setNeutralMode(mode);
    }
  }

  public void resetEncoder() {
    master.setSelectedSensorPosition(0);
  }

  protected void addFollower(TalonFX follower, FollowerType followerType, InvertType invertType) {
    follower.follow(master, followerType);
    follower.setInverted(invertType);
    followers.add(follower);
  }
}
