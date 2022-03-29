package team3647.frc2022.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import team3647.lib.PeriodicSubsystem;

public class Ballstopper implements PeriodicSubsystem {
    private final Solenoid piston;

    public Ballstopper(Solenoid piston) {
        this.piston = piston;
    }

    public void extend() {
        set(true);
    }

    public void retract() {
        set(false);
    }

    public void set(boolean value) {
        // piston.set(value);
    }

    @Override
    public String getName() {
        return "BallStopper";
    }
}
