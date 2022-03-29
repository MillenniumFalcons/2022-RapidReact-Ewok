package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import team3647.lib.NetworkColorSensor;
import team3647.lib.NetworkColorSensor.Color;
import team3647.lib.TalonFXSubsystem;

public class ColumnBottom extends TalonFXSubsystem {
    private final NetworkColorSensor colorSensor;

    private final SimpleMotorFeedforward ff;

    public ColumnBottom(
            TalonFX master,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt,
            SimpleMotorFeedforward ff,
            NetworkColorSensor colorSensor) {
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);
        setStatusFramesThatDontMatter(master, kLongStatusTimeMS, kTimeoutMS);
        this.ff = ff;
        this.colorSensor = colorSensor;
    }

    /** @param vel velocity in m/s positive is up (towards the shooter) */
    public void setSurfaceVelocity(double vel) {
        setVelocity(vel, ff.calculate(getVelocity(), vel, kDt));
    }

    public Color getBallColor() {
        return colorSensor.getColor();
    }

    public NetworkColorSensor getColorSensor() {
        return this.colorSensor;
    }

    public boolean isBallWithinDistance() {
        return colorSensor.isReadColor();
    }

    public void stop() {
        setOpenloop(0);
    }

    @Override
    public String getName() {
        return "Column Bottom";
    }
}
