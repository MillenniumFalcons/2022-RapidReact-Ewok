package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import team3647.lib.TalonFXSubsystem;

public class ClimberArm extends TalonFXSubsystem {

    public ClimberArm(
            TalonFX master,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt) {
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);
    }

    public void moveMotionMagic(double length) {
        super.setPositionMotionMagic(length, 0);
    }

    /**
     * @param position
     * @param kG voltage needed to overcome gravity with robot hanging
     */
    public void holdAtPosition(double position, double kG) {
        super.setPosition(position, kG);
    }

    @Override
    public String getName() {
        // TODO Auto-generated method stub
        return null;
    }
}
