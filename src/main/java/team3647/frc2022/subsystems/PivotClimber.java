package team3647.frc2022.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Solenoid;
import team3647.lib.PeriodicSubsystem;

public class PivotClimber implements PeriodicSubsystem {
    public static enum ClimberAngle {
        ANGLED(true),
        STRAIGHT(false);
        public final boolean solenoidVal;

        ClimberAngle(boolean solenoidVal) {
            this.solenoidVal = solenoidVal;
        }
    }

    private final Solenoid pivotPistons;

    private final double maxLengthAngled;
    private final double maxLengthStraight;
    private final double voltageToHoldRobot;
    public final ClimberArm leftArm;
    public final ClimberArm rightArm;
    private ClimberAngle climberAngle;

    public PivotClimber(
            ClimberArm leftArm,
            ClimberArm rightArm,
            Solenoid pivotPistons,
            double maxLengthAngled,
            double maxLengthStraight,
            double voltageToHoldRobot) {

        this.pivotPistons = pivotPistons;
        this.maxLengthAngled = maxLengthAngled;
        this.maxLengthStraight = maxLengthStraight;
        this.voltageToHoldRobot = voltageToHoldRobot;
        this.leftArm = leftArm;
        this.rightArm = rightArm;
        setState(ClimberAngle.STRAIGHT);
    }

    public void setAngled() {
        setState(ClimberAngle.ANGLED);
    }

    public void setStraight() {
        setState(ClimberAngle.STRAIGHT);
    }

    public void setState(ClimberAngle angle) {
        climberAngle = angle;
    }

    public void moveMotionMagic(double length) {
        double maxLength =
                ClimberAngle.STRAIGHT.equals(climberAngle) ? maxLengthStraight : maxLengthAngled;
        length = MathUtil.clamp(length, 0, maxLength);
        leftArm.moveMotionMagic(length);
        rightArm.moveMotionMagic(length);
    }

    /**
     * Holds position at the set position
     *
     * @param position
     */
    public void holdPosition(double position) {
        leftArm.holdAtPosition(position, voltageToHoldRobot);
        rightArm.holdAtPosition(position, voltageToHoldRobot);
    }

    public void setOpenloop(double demand) {
        leftArm.setOpenloop(demand);
        rightArm.setOpenloop(demand);
    }

    public double getLeftPosition() {
        return leftArm.getPosition();
    }

    public double getRightPosition() {
        return rightArm.getPosition();
    }

    public void setToBrake() {
        leftArm.setToBrake();
        rightArm.setToBrake();
    }

    @Override
    public void writePeriodicOutputs() {
        pivotPistons.set(climberAngle.solenoidVal);
    }

    @Override
    public String getName() {
        return "Pivot Climber";
    }
}
