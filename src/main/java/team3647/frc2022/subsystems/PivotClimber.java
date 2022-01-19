package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Solenoid;
import team3647.lib.TalonFXSubsystem;

public class PivotClimber extends TalonFXSubsystem {
    private final Solenoid pivotPistons;
    private final SimpleMotorFeedforward ff;

    private final double minLength;
    private final double maxLength;

    public PivotClimber(
            TalonFX leftMaster,
            TalonFX rightMaster,
            Solenoid pivotPistons,
            SimpleMotorFeedforward ff,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt,
            double minLength,
            double maxLength) {
        super(leftMaster, velocityConversion, positionConversion, nominalVoltage, kDt);
        addFollower(rightMaster, FollowerType.PercentOutput, InvertType.InvertMotorOutput);
        this.ff = ff;
        this.pivotPistons = pivotPistons;
        this.minLength = minLength;
        this.maxLength = maxLength;
        setToBrake();
    }

    public void setAngled() {
        this.pivotPistons.set(true);
    }

    public void setStraight() {
        this.pivotPistons.set(false);
    }

    public void setLengthMotionMagic(double length) {
        super.setPositionMotionMagic(MathUtil.clamp(length, minLength, maxLength), 0);
    }

    public void manualRetract(double percentOutput) {
        super.setOpenloop(percentOutput);
    }

    @Override
    public String getName() {

        // TODO Auto-generated method stub
        return "Pivot Climber";
    }
}
