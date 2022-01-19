package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import team3647.lib.TalonFXSubsystem;

public class Flywheel extends TalonFXSubsystem {
    private final SimpleMotorFeedforward ff;
    private double surfaceSpeed;
    private final double RPMtoMetersPerSecond;

    public Flywheel(
            TalonFX master,
            TalonFX follower,
            SimpleMotorFeedforward ff,
            double velocityConversion,
            double positionConversion,
            double RPMtoMetersPerSecond) {
        super(master, velocityConversion, positionConversion);
        addFollower(follower, FollowerType.PercentOutput, InvertType.InvertMotorOutput);
        this.ff = ff;
        this.RPMtoMetersPerSecond = RPMtoMetersPerSecond;
    }

    public void setFlywheelRPM(double RPM) {
        double feedforward = ff.calculate(RPM);
        setVelocity(RPM, feedforward);
    }

    public void setFlywheelSurfaceSpeed(double MPS) {
        double demand = MPS / RPMtoMetersPerSecond;
        double feedforward = ff.calculate(demand);
        setVelocity(demand, feedforward);
    }

    @Override
    public void readPeriodicInputs() {
        super.readPeriodicInputs();
        surfaceSpeed = getVelocity() * RPMtoMetersPerSecond;
    }

    public double getSurfaceSpeed() {
        return surfaceSpeed;
    }

    @Override
    public String getName() {
        return "Flywheel";
    }
}
