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

        double currentPosition = getPosition(); // returns [-200,200]
        angle -= 360.0 * Math.round(angle / 360.0); // angles in [-180, 180]
        double targetPosition = angle;
        boolean targetInOverlap =
                (angle >= minAngle + 360 && angle <= 180)
                        || (angle <= maxAngle - 360 && angle >= -180);

        /*Convert target angle to pick the shortest rotate direction if target angle lies in [160,180] or [-180,-160]*/
        if (targetInOverlap) {

            if (targetPosition > currentPosition) {
                /*For example, target angle is 170 while current angle is -200, make target -190 instead */
                if (targetPosition > currentPosition + 180) {
                    targetPosition -= 360;
                }
            } else {
                /*For example, target angle is -170 while current angle is 90, make target 190 insteadd */
                if (targetPosition < currentPosition - 180) {
                    targetPosition += 360;
                }
            }
        }

        setPosition(targetPosition, staticFrictionVolts);
    }

    /*return angle in[-180,180]*/
    public double getAngle() {
        double angle = getPosition();
        angle -= 360.0 * Math.round(angle / 360.0);
        return angle;
    }

    public boolean getLimitSwitchValue() {
        return limitSwitch.get();
    }

    @Override
    public String getName() {
        return "Turret";
    }
}
