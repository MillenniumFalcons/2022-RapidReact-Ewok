package team3647.frc2022.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import java.util.function.DoubleSupplier;
import team3647.frc2022.commands.ClimberCommands;
import team3647.frc2022.commands.ColumnTopCommands;
import team3647.frc2022.commands.FeederCommands;
import team3647.frc2022.commands.FlywheelCommands;
import team3647.frc2022.commands.HoodCommands;
import team3647.frc2022.commands.IntakeCommands;
import team3647.frc2022.commands.turret.AimTurret;
import team3647.frc2022.commands.turret.TurretCommands;
import team3647.frc2022.constants.FlywheelConstants;
import team3647.frc2022.constants.HoodContants;
import team3647.frc2022.states.RobotState;
import team3647.lib.tracking.FlightDeck;
import team3647.lib.vision.AimingParameters;

public class Superstructure {
    private final FlightDeck deck;
    private final PivotClimber m_climber;
    private final ColumnBottom m_columnBottom;
    private final VerticalRollers m_verticalRollers;
    private final ColumnTop m_columnTop;
    private final Intake m_intake;
    private final Turret m_turret;
    private final Hood m_hood;
    private final Flywheel m_flywheel;
    private final Ballstopper stopper;

    public final FlywheelCommands flywheelCommands;
    public final HoodCommands hoodCommands;
    public final ClimberCommands climberCommands;
    public final TurretCommands turretCommands;
    public final ColumnTopCommands columnTopCommands;
    public final FeederCommands feederCommands;
    public final IntakeCommands intakeCommands;

    private RobotState currentState, aimedState;

    private AimingParameters aimingParameters;
    private double flywheelVelocity = 0;
    private double kickerVelocity = 0;
    private double hoodAngle = 16;

    public Superstructure(
            FlightDeck deck,
            PivotClimber m_climber,
            ColumnBottom m_columnBottom,
            VerticalRollers m_verticalRollers,
            ColumnTop m_columnTop,
            Intake m_intake,
            Turret m_turret,
            Hood m_hood,
            Flywheel m_flywheel,
            Ballstopper stopper) {
        this.deck = deck;
        this.m_climber = m_climber;
        this.m_columnBottom = m_columnBottom;
        this.m_verticalRollers = m_verticalRollers;
        this.m_columnTop = m_columnTop;
        this.m_intake = m_intake;
        this.m_turret = m_turret;
        this.m_hood = m_hood;
        this.m_flywheel = m_flywheel;
        this.stopper = stopper;
        this.aimedState = RobotState.STOPPED;

        flywheelCommands = new FlywheelCommands(m_flywheel);
        hoodCommands = new HoodCommands(m_hood);
        climberCommands = new ClimberCommands(m_climber);
        columnTopCommands = new ColumnTopCommands(m_columnTop);
        feederCommands = new FeederCommands(m_columnBottom, m_columnTop, m_verticalRollers);
        intakeCommands = new IntakeCommands(m_intake);
        turretCommands = new TurretCommands(m_turret);
    }

    public Command getSpinupCommand() {
        return flywheelCommands.getVariableAccelerateFlywheel(
                () -> FlywheelConstants.getFlywheelRPM(getDistanceToTarget()));
    }

    public Command getShootCommand() {
        return flywheelCommands
                .getGoVariableVelocity(this::getAimedFlywheelSurfaceVel)
                .alongWith(
                        columnTopCommands.getGoVariableVelocity(this::getAimedKickerVelocity),
                        getWaitForShooter()
                                .andThen(
                                        getRetractStopper(),
                                        feederCommands
                                                .getFeedInwardsUntil(
                                                        this::getFlywheelNotAtSurfaceVel)
                                                .andThen(getDeployStopper(), getWaitForShooter()))
                                .andThen(
                                        getRetractStopper(),
                                        feederCommands.getFeedInwardsUntil(
                                                this::getFlywheelNotAtSurfaceVel))
                                .andThen(getDeployStopper()));
    }

    public Command intakeAndIndex(DoubleSupplier percentOut) {
        return getDeployStopper()
                .andThen(
                        getIntakeSequence(percentOut)
                                .alongWith(
                                        feederCommands.getRunColmnBottomInwards(),
                                        feederCommands.getRunVerticalRollers()));
    }

    public Command getAutoClimbSequence() {
        return new ConditionalCommand(
                        climberCommands.getClimberToNextRung(),
                        new InstantCommand(() -> setState(RobotState.CLIMB))
                                .andThen(
                                        turretCommands
                                                .getTurretMotionMagic(180)
                                                .andThen(climberCommands.getClimberDeploy())),
                        this::isClimbing)
                .alongWith(new ScheduleCommand(flywheelCommands.stopFlywheel()));
    }

    public Command getHoodAdjustUnlessClimbing() {
        return new ConditionalCommand(
                new RunCommand(() -> m_hood.setOpenloop(0), m_hood),
                getHoodAutoAdjustCommand(),
                this::isClimbing);
    }

    public Command getAutoAimAndShoot() {
        return getAimTurretCommand().alongWith(getShootCommand());
    }

    public Command getHoodAutoAdjustCommand() {
        return hoodCommands.getAutoAdjustHood(this::getAimedHoodAngle);
    }

    public Command getIntakeSequence(DoubleSupplier percentOut) {
        return intakeCommands.getIntakeSequnce(intakeCommands.getRunIntakeOpenloop(percentOut));
    }

    public Command getExtendClimberManual() {
        return new ConditionalCommand(
                climberCommands.setAngled(), new InstantCommand(), this::isClimbing);
    }

    public Command getRetractClimbManual() {
        return new ConditionalCommand(
                climberCommands.setStraight(), new InstantCommand(), this::isClimbing);
    }

    public Command getAimTurretCommand() {
        return new ConditionalCommand(
                new InstantCommand(),
                new AimTurret(
                        m_turret,
                        this::getAimingParameters,
                        deck.getTracker()::getMeasuredVelocity),
                this::isClimbing);
    }

    public Command getClimberManualControl(DoubleSupplier percentOut) {
        return new ConditionalCommand(
                climberCommands.getClimberOpenloop(percentOut),
                new InstantCommand(),
                this::isClimbing);
    }

    public Command getWaitForShooter() {
        return new WaitUntilCommand(this::getFlywheelAtSurfaceVel);
    }

    public Command getDeployStopper() {
        return new InstantCommand(stopper::extend);
    }

    public Command getRetractStopper() {
        return new InstantCommand(stopper::retract);
    }

    public double getDistanceToTarget() {
        if (aimingParameters == null) {
            return 0;
        }
        return aimingParameters.getRangeMeters();
    }

    public boolean getFlywheelAtSurfaceVel() {
        return m_flywheel.getVelocity() > getAimedFlywheelSurfaceVel();
    }

    public boolean getFlywheelNotAtSurfaceVel() {
        return !getFlywheelAtSurfaceVel();
    }

    public double getAimedFlywheelSurfaceVel() {
        return flywheelVelocity;
    }

    public double getAimedKickerVelocity() {
        return kickerVelocity;
    }

    public double getAimedHoodAngle() {
        return hoodAngle;
    }

    public Command getIntakeHoldCommand(DoubleSupplier output) {
        return intakeCommands
                .getIntakeSequnce(
                        intakeCommands.getIntakeSequnce(
                                intakeCommands.getRunIntakeOpenloop(output)))
                .alongWith(
                        new ConditionalCommand(
                                feederCommands.getFeedInwardsUntil(
                                        m_columnBottom::getMiddleBannerValue),
                                feederCommands.getFeedInwards(),
                                m_columnTop::getTopBannerValue));
    }

    public Command getIntakeReleaseCommand() {
        return feederCommands
                .getFeedOutwardsUntil(m_columnTop::getNotTopBannerValue)
                .andThen(
                        intakeCommands.getEndSequence().alongWith(feederCommands.getEndSequence()));
    }

    public AimingParameters getAimingParameters() {
        return aimingParameters;
    }

    public void setState(RobotState state) {
        this.aimedState = state;
    }

    public void periodic(double timestamp) {
        aimingParameters = deck.getLatestParameters();
        if (aimingParameters != null) {
            flywheelVelocity = FlywheelConstants.getFlywheelRPM(aimingParameters.getRangeMeters());
            kickerVelocity = flywheelVelocity * 0.5;
            hoodAngle = HoodContants.getHoodAngle(aimingParameters.getRangeMeters());
        }
    }

    public boolean isClimbing() {
        return RobotState.CLIMB.equals(aimedState);
    }
}
