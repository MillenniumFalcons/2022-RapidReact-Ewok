package team3647.frc2022.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import team3647.lib.PeriodicSubsystem;

public class BallStopper implements PeriodicSubsystem {
    private final Solenoid stopPistons;
    private boolean extend;

    public BallStopper(Solenoid stopPistons) {
        this.stopPistons = stopPistons;
        retract();
    }

    public void extend() {
        extend = true;
    }

    public void retract() {
        extend = false;
    }

    public void set(boolean value) {
        extend = value;
    }

    @Override
    public void writePeriodicOutputs() {
        stopPistons.set(extend);
    }

    @Override
    public String getName() {
        return "BallStopper";
    }
}
