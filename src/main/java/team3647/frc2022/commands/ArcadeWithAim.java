package team3647.frc2022.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import team3647.frc2022.subsystems.Drivetrain;

public class ArcadeWithAim extends CommandBase {

    private final Drivetrain m_drivetrain;
    private final DoubleSupplier m_throttle;
    private final DoubleSupplier m_turn;
    private final BooleanSupplier m_quickTurn;
    private final BooleanSupplier m_doAim;
    private final SlewRateLimiter m_accelerationLimiter = new SlewRateLimiter(2);
    private final Supplier<Translation2d> m_target;

    private final double driveMultiplier = 1;
    private final double turnMultiplier = 1;

    public ArcadeWithAim(
            Drivetrain drivetrain,
            DoubleSupplier throttle,
            DoubleSupplier turn,
            BooleanSupplier quickTurn,
            BooleanSupplier doAim,
            Supplier<Translation2d> target) {
        m_drivetrain = drivetrain;
        m_throttle = throttle;
        m_turn = turn;
        m_quickTurn = quickTurn;
        m_doAim = doAim;
        m_target = target;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double throttle = m_throttle.getAsDouble() * driveMultiplier;
        double turn = m_turn.getAsDouble() * turnMultiplier;
        if (m_doAim.getAsBoolean()) {
            double xDesplacment = m_drivetrain.getPose().getX() - m_target.get().getX();
            double yDesplacment = m_drivetrain.getPose().getY() - m_target.get().getY();
            double theta = Math.atan(xDesplacment / yDesplacment);
            turn = theta * (2 / Math.PI);
        }
        m_drivetrain.curvatureDrive(
                m_accelerationLimiter.calculate(throttle * throttle * Math.signum(throttle)),
                turn * turn * Math.signum(turn) * 0.8,
                m_quickTurn.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
