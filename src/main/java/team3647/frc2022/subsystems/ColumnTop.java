package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import team3647.lib.TalonFXSubsystem;

public class ColumnTop extends TalonFXSubsystem {
    private final SimpleMotorFeedforward ff;

    public ColumnTop(
            TalonFX master,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt,
            SimpleMotorFeedforward ff) {
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);
        setStatusFramesThatDontMatter(master, kLongStatusTimeMS);
        this.ff = ff;
        setToCoast();
    }

    /** @param vel velocity in m/s positive is up (towards the shooter) */
    public void setSurfaceVelocity(double vel) {
        // setVelocity(vel, ff.calculate(getVelocity(), vel, kDt));
        setVelocity(vel, 0);
    }

    @Override
    public String getName() {
        return "Column Top";
    }
}
