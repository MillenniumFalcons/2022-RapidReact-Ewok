package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Solenoid;
import team3647.lib.TalonFXSubsystem;

public class PivotClimber extends TalonFXSubsystem {
    public static enum ClimberAngle {
        ANGLED(true),
        STRAIGHT(false);
        public final boolean solenoidVal;

        ClimberAngle(boolean solenoidVal) {
            this.solenoidVal = solenoidVal;
        }
    }

    private final Solenoid pivotPistons;

    private final double minLength;
    private final double maxLengthAngled;
    private final double maxLengthStraight;
    private final double voltageToHoldRobot;
    private ClimberAngle climberAngle;

    public PivotClimber(
            TalonFX leftMaster,
            TalonFX rightMaster,
            Solenoid pivotPistons,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt,
            double minLength,
            double maxLengthAngled,
            double maxLengthStraight,
            double voltageToHoldRobot) {
        super(leftMaster, velocityConversion, positionConversion, nominalVoltage, kDt);
        addFollower(rightMaster, FollowerType.PercentOutput, InvertType.InvertMotorOutput);
        this.pivotPistons = pivotPistons;
        this.minLength = minLength;
        this.maxLengthAngled = maxLengthAngled;
        this.maxLengthStraight = maxLengthStraight;
        this.voltageToHoldRobot = voltageToHoldRobot;
        setState(ClimberAngle.STRAIGHT);
        setToBrake();
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

    public void setLengthMotionMagic(double length) {
        double maxLength =
                ClimberAngle.STRAIGHT.equals(climberAngle) ? maxLengthStraight : maxLengthAngled;
        super.setPositionMotionMagic(MathUtil.clamp(length, minLength, maxLength), 0);
    }

    /**
     * Holds position at the set position
     *
     * @param position
     */
    public void holdPosition(double position) {
        super.setPosition(position, voltageToHoldRobot);
    }

    @Override
    public void writePeriodicOutputs() {
        super.writePeriodicOutputs();
        pivotPistons.set(climberAngle.solenoidVal);
    }

    @Override
    public String getName() {

        // TODO Auto-generated method stub
        return "Pivot Climber";
    }
}
