package team3647.frc2022.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import team3647.lib.TalonFXSubsystem;

public class ColumnBottom extends TalonFXSubsystem {
    private final DigitalInput bottomBanner;
    private final DigitalInput middleBanner;
    private final SimpleMotorFeedforward ff;

    public boolean bottomBannerValue;
    public boolean middleBannerValue;

    public ColumnBottom(
            TalonFX master,
            DigitalInput bottomBanner,
            DigitalInput middleBanner,
            double velocityConversion,
            double positionConversion,
            double nominalVoltage,
            double kDt,
            SimpleMotorFeedforward ff) {
        super(master, velocityConversion, positionConversion, nominalVoltage, kDt);
        setStatusFramesThatDontMatter(master, kLongStatusTimeMS, kTimeoutMS);
        this.bottomBanner = bottomBanner;
        this.middleBanner = middleBanner;
        this.bottomBannerValue = false;
        this.middleBannerValue = false;
        this.ff = ff;
    }

    /** @param vel velocity in m/s positive is up (towards the shooter) */
    public void setSurfaceVelocity(double vel) {
        setVelocity(vel, ff.calculate(getVelocity(), vel, kDt));
    }

    @Override
    public void readPeriodicInputs() {
        super.readPeriodicInputs();
        bottomBannerValue = bottomBanner.get();
        middleBannerValue = middleBanner.get();
    }
    /** @return bottomBannerValue returns false when no ball, returns true when ball */
    public boolean getBottomBannerValue() {
        return !bottomBannerValue;
    }
    /** @return middleBannerValue returns false when no ball, returns true when ball */
    public boolean getMiddleBannerValue() {
        return !middleBannerValue;
    }

    public void stop() {
        setOpenloop(0);
    }

    @Override
    public String getName() {
        return "Column Bottom";
    }
}
