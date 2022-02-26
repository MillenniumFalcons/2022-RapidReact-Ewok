package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import team3647.lib.TalonFXSubsystem;

public class Turret extends TalonFXSubsystem {

    private final double maxAngle;
    private final double minAngle;
    private final SimpleMotorFeedforward ff;
    private final double kS;

    public Turret(
            TalonFX master,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt,
            double kS,
            double maxAngle,
            double minAngle,
            SimpleMotorFeedforward ff) {
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);
        setStatusFramesThatDontMatter(master, kLongStatusTimeMS, kTimeoutMS);
        this.maxAngle = maxAngle;
        this.minAngle = minAngle;
        this.ff = ff;
        this.kS = kS;
        resetEncoder();
    }

    /** @param angle in degree, [-180,180] */
    public void setAngle(double angle, double velocity) {

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

        // Multiply the static friction volts by -1 if our target position is less than current
        // position; if we need to move backwards, the volts needs to be negative
        double ffVolts = ff.calculate(velocity);
        if (ffVolts != 0) {
            ffVolts = kS;
        }

        setPosition(targetPosition, ffVolts);
    }

    public void setAngleMotionMagic(double angle) {
        setAngle(angle, 0);
        // throw new UnsupportedOperationException("Method unimplemented yet");
    }

    /** @return angle in [-180,180] */
    public double getAngle() {
        double angle = getPosition();
        angle -= 360.0 * Math.round(angle / 360.0);
        return angle;
    }

    @Override
    public void periodic() {
        // readPeriodicInputs(); Call in Robot.addPeriodic
        writePeriodicOutputs();
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(getAngle());
    }

    @Override
    public String getName() {
        return "Turret";
    }
}
