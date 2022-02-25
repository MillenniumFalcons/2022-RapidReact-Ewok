package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import team3647.lib.TalonFXSubsystem;

public class ColumnTop extends TalonFXSubsystem {
    private final DigitalInput topBanner;
    private final SimpleMotorFeedforward ff;

    public boolean topBannerValue;

    public ColumnTop(
            TalonFX master,
            DigitalInput topBanner,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt,
            SimpleMotorFeedforward ff) {
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);
        setStatusFramesThatDontMatter(master, kLongStatusTimeMS, kTimeoutMS);
        this.topBanner = topBanner;
        this.topBannerValue = false;
        this.ff = ff;
        setToCoast();
    }

    /** @param vel velocity in m/s positive is up (towards the shooter) */
    public void setSurfaceVelocity(double vel) {
        setVelocity(vel, ff.calculate(vel));
    }

    @Override
    public void readPeriodicInputs() {
        super.readPeriodicInputs();
        topBannerValue = topBanner.get();
    }
    /** @return topbannerValue returns false when no ball, returns true when ball */
    public boolean getTopBannerValue() {
        return !topBannerValue;
    }

    public boolean getNotTopBannerValue() {
        return topBannerValue;
    }

    @Override
    public String getName() {
        return "Column Top";
    }
}
