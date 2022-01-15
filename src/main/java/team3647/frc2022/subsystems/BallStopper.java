package team3647.frc2022.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import team3647.lib.PeriodicSubsystem;

public class BallStopper implements PeriodicSubsystem {
    private final Solenoid stopPistons;

    public BallStopper(Solenoid stopPistons) {
        this.stopPistons = stopPistons;
    }

    public void extend() {
        set(true);
    }

    public void retract() {
        set(false);
    }

    public void set(boolean value) {
        stopPistons.set(value);
    }

    @Override
    public String getName() {
        return "BallStopper";
    }
}
