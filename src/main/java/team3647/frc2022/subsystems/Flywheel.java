package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import team3647.lib.TalonFXSubsystem;

public class Flywheel extends TalonFXSubsystem {
    private final SimpleMotorFeedforward ff;
    private double RPMtoMetersPerSecond;
    private double metersPerSecond;

    public Flywheel(
            TalonFX master,
            TalonFX follower,
            SimpleMotorFeedforward ff,
            double velocityConversion,
            double positionConversion) {
        super(master, velocityConversion, positionConversion);
        addFollower(follower, FollowerType.PercentOutput, InvertType.InvertMotorOutput);
        this.ff = ff;
        // setToBrake();
    }

    public void setFlywheelRPM(double demand) {
        double feedforward = ff.calculate(demand);
        setVelocity(demand, feedforward);
    }

    @Override
    public void readPeriodicInputs() {
        super.readPeriodicInputs();
        metersPerSecond = getVelocity() * RPMtoMetersPerSecond;
    }

    public double getMetersPerSecond() {
        return metersPerSecond;
    }

    public void setToBrake() {
        super.setToBrake();
    }

    @Override
    public String getName() {
        return "Flywheel";
    }
}
