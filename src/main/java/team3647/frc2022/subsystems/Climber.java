package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import team3647.lib.TalonFXSubsystem;

public class Climber extends TalonFXSubsystem {

    public Climber(TalonFX left) {
        super(left, 0, 0);
    }

    @Override
    public String getName() {

        // TODO Auto-generated method stub
        return null;
    }
}
