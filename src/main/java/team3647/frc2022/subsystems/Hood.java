package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import team3647.lib.TalonFXSubsystem;

public class Hood extends TalonFXSubsystem {
    private final double minPos;
    private final double maxPos;

    public Hood(
            TalonFX master,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt,
            double minPos,
            double maxPos) {
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);
        this.minPos = minPos;
        this.maxPos = maxPos;
        setStatusFramesThatDontMatter(master, kLongStatusTimeMS);
    }

    public void setAngleMotionMagic(double angle) {
        super.setPositionMotionMagic(MathUtil.clamp(angle, minPos, maxPos), 0);
    }

    public void setAngle(double angle) {
        super.setPosition(MathUtil.clamp(angle, minPos, maxPos), 0);
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
