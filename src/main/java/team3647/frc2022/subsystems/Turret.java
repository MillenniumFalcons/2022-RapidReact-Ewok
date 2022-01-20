package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import team3647.lib.TalonFXSubsystem;

public class Turret extends TalonFXSubsystem {

    private final double maxAngle;
    private final double minAngle;
    private final SimpleMotorFeedforward ff;
    private final double positionConversion;
    private final DigitalInput limitSwitch;

    public Turret(
            TalonFX master,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt,
            double maxAngle,
            double minAngle,
            SimpleMotorFeedforward ff,
            int limitSwitchPin) {
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);
        this.maxAngle = maxAngle;
        this.minAngle = minAngle;
        this.positionConversion = positionConversion;
        this.ff = ff;
        limitSwitch = new DigitalInput(limitSwitchPin);
        
        

    }
    // manually control the turret
    //public void setOpenloop(double demand) {
        //setOpenloop(demand);
   // }
    // position feedforward
    // https://github.com/MillenniumFalcons/2021-InfiniteRecharge-Offseason/blob/b3ea4822218feef30e609dc408b3ec652a179025/src/main/java/team3647/frc2020/subsystems/TalonSRXSubsystem.java#L160
    public void setAngle(double angle) {

        double curretPosition = getPosition(); // returns [-200,200]
        double targetPosition;

        angle -= 360.0*Math.round(angle/360.0); // angles in [-180, 180]
        

        double clockwiseDistance;
        double counterClockwiseDistance;
        double demand;

        if(angle > curretPosition){
            counterClockwiseDistance = Math.abs(angle - curretPosition);
            clockwiseDistance = Math.abs((360-angle)+curretPosition);
            if (){
                
            }
        }

        setPosition(
                targetPosition,
                ff.calculate(Math.signum(demand * positionConversion - getPosition())));

    }

    public double getAngle() {
        return getPosition();
    }

    @Override
    public String getName() {
        return "Turret";
    }
}
