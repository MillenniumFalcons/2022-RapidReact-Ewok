// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import team3647.lib.PeriodicSubsystem;

public class Ballstopper implements PeriodicSubsystem {
    private final Solenoid stopPistons;

    public Ballstopper(Solenoid stopper) {
        stopPistons = stopper;
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
