package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import team3647.lib.TalonFXSubsystem;

public class Hood extends TalonFXSubsystem {
    private final double minPosDeg;
    private final double maxPosDeg;
    private final double posThresholdDeg;
    private final double kS;

    public Hood(
            TalonFX master,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kS,
            double kDt,
            double minPosDeg,
            double maxPosDeg,
            double posThresholdDeg) {
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);
        this.minPosDeg = minPosDeg;
        this.maxPosDeg = maxPosDeg;
        this.posThresholdDeg = posThresholdDeg;
        this.kS = kS;
        setStatusFramesThatDontMatter(master, kLongStatusTimeMS, kTimeoutMS);
        resetEncoder();
    }

    public void setAngleMotionMagic(double angle) {
        double multiplier = Math.signum(angle - getAngle()) * 0.575;
        super.setPositionMotionMagic(
                MathUtil.clamp(angle, minPosDeg + posThresholdDeg, maxPosDeg - posThresholdDeg),
                kS * multiplier);
    }

    public void setAngle(double angle) {
        super.setPosition(
                MathUtil.clamp(angle, minPosDeg + posThresholdDeg, maxPosDeg - posThresholdDeg), 0);
    }

    public double getAngle() {
        return super.getPosition();
    }

    @Override
    public void resetEncoder() {
        System.out.println("Min position Degree: " + minPosDeg);
        super.setEncoder(minPosDeg);
    }

    @Override
    public String getName() {
        return "Hood";
    }
}
