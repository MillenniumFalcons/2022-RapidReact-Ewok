package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import team3647.lib.TalonFXSubsystem;

public class Hood extends TalonFXSubsystem {
    private final DigitalInput limitSwitch;
    private final double minPos;
    private final double maxPos;

    public Hood(TalonFX master, DigitalInput revLimitSwitch, double minPos, double maxPos, double velocityConversion,
    double positionConversion) {
        super(master, velocityConversion, positionConversion);
        this.limitSwitch = revLimitSwitch;
        this.minPos = minPos;
        this.maxPos = maxPos;
    }

    public void setAngleMotionMagic(double angle) {
        super.setPositionMotionMagic(MathUtil.clamp(angle, minPos, maxPos), 0);
    }

    public void setAngle(double angle) {
        super.setPosition(MathUtil.clamp(angle, minPos, maxPos), 0);
    }

    public boolean getLimitSwitchValue() {
        return limitSwitch.get();
    }

    @Override
    public void resetEncoder() {
        super.setEncoder(minPos);
    }

    @Override
    public String getName() {
        return "Hood";
    }
}
