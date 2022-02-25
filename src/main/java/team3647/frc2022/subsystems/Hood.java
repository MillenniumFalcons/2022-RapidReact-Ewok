package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.MedianFilter;
import team3647.lib.TalonFXSubsystem;

public class Hood extends TalonFXSubsystem {
    private final double minPosDeg;
    private final double maxPosDeg;
    private final double posThresholdDeg;
    private final double kS;
    private MedianFilter filter;
    private double medianVel;

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
        this.filter = new MedianFilter(10);
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

    public boolean hitForwardHardstop() {
        return this.getDemand() > 0 && medianVel < 0.01;
    }

    public boolean hitReverseHardstop() {
        return this.getDemand() < 0 && medianVel < 0.01;
    }

    public void resetEncoderForward() {
        System.out.println("Max position Degree: " + maxPosDeg);
        super.setEncoder(maxPosDeg);
    }

    @Override
    public void readPeriodicInputs() {
        super.readPeriodicInputs();
        medianVel = filter.calculate(this.getVelocity());
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
