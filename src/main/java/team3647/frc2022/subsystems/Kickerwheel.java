package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import team3647.lib.TalonFXSubsystem;

public class Kickerwheel extends TalonFXSubsystem {
    private final SimpleMotorFeedforward ff;

    public Kickerwheel(
            TalonFX master,
            SimpleMotorFeedforward ff,
            double velocityConversion,
            double positionConversion) {
        super(master, velocityConversion, positionConversion);
        this.ff = ff;
    }

    public void setKickerRPM(double demand) {
        // TODO: acceleration in ff
        // PIO with timestamps and measure current and last velocity
        double feedforward = ff.calculate(demand);
        setVelocity(demand, feedforward);
    }

    @Override
    public String getName() {
        return "Kickerwheel";
    }
}
