package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import team3647.lib.TalonFXSubsystem;

public class Turret extends TalonFXSubsystem {

    private final double maxAngle;
    private final double minAngle;
    private final DigitalInput limitSwitch;
    private final double staticFrictionVolts;

    public Turret(
            TalonFX master,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt,
            double maxAngle,
            double minAngle,
            int limitSwitchPin,
            double staticFrictionVolts) {
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);
        this.maxAngle = maxAngle;
        this.minAngle = minAngle;
        limitSwitch = new DigitalInput(limitSwitchPin);
        this.staticFrictionVolts = staticFrictionVolts;
        resetEncoder();
    }

    /** @param angle in degree, [-180,180] */
    public void setAngle(double angle) {

        double currentAngle = getAngle(); // returns [-200,200]
        angle -= 360.0 * Math.round(angle / 360.0); // angles in [-180, 180]
        double targetAngle = angle;

        /*Convert target angle to pick the shortest rotate direction if target angle lies in [160,180] or [-180,-160]*/
        if ((angle >= minAngle + 360 && angle <= 180)
                || (angle <= maxAngle - 360 && angle >= -180)) {

            if (targetAngle > currentAngle) {
                /*For example, target angle is 170 while current angle is -200, make target -190 instead */
                if (targetAngle > currentAngle + 180) {
                    targetAngle -= 360;
                }
            } else {
                /*For example, target angle is -170 while current angle is 90, make target 190 insteadd */
                if (targetAngle < currentAngle - 180) {
                    targetAngle += 360;
                }
            }
        }

        setPosition(targetAngle, staticFrictionVolts);
    }

    public double getAngle() {
        return getPosition();
    }

    public boolean getLimitSwitchValue() {
        return limitSwitch.get();
    }

    @Override
    public String getName() {
        return "Turret";
    }
}
