package team3647.frc2022.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import team3647.frc2022.constants.LEDConstants;
import team3647.frc2022.states.ClimberState;
import team3647.frc2022.states.RobotState;
import team3647.frc2022.states.ShooterState;
import team3647.lib.tracking.FlightDeck;
import team3647.lib.vision.AimingParameters;

public class Superstructure {
    public RobotState currentState = new RobotState();

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
            Ballstopper ballstopper,
            StatusLED statusLEDs) {
        this.deck = deck;
        this.m_climber = m_climber;
        this.m_columnBottom = m_columnBottom;
        this.m_verticalRollers = m_verticalRollers;
        this.m_columnTop = m_columnTop;
        this.m_intake = m_intake;
        this.m_turret = m_turret;
        this.m_hood = m_hood;
        this.m_flywheel = m_flywheel;
        this.m_ballstopper = ballstopper;
        this.m_statusLED = statusLEDs;

        flywheelCommands = new FlywheelCommands(m_flywheel);
        hoodCommands = new HoodCommands(m_hood);
        climberCommands = new ClimberCommands(m_climber);
        columnTopCommands = new ColumnTopCommands(m_columnTop);
        feederCommands =
                new FeederCommands(m_columnBottom, m_columnTop, m_verticalRollers, m_ballstopper);
        intakeCommands = new IntakeCommands(m_intake);
        turretCommands = new TurretCommands(m_turret);
        isClimbing = new Trigger(this::isClimbing);
        hasTargetTrigger = new Trigger(this::hasTarget);
        readyToShootTrigger = new Trigger(this::getFlywheelReady);
        newTargetTrigger = new Trigger(this::hasNewTarget);
        isShooting = new Trigger(this::isShooting);

        hasTargetTrigger
                .negate()
                .whenActive(() -> m_statusLED.setAnimation(LEDConstants.kBlinkRed), m_statusLED);
        hasTargetTrigger
                .and(newTargetTrigger)
                .and(readyToShootTrigger)
                .and(isClimbing.negate())
                .whenActive(() -> m_statusLED.setAnimation(LEDConstants.kBlinkGreen), m_statusLED);
        hasTargetTrigger
                .and(newTargetTrigger)
                .and(isClimbing.negate())
                .and(readyToShootTrigger.negate())
                .whenActive(() -> m_statusLED.setAnimation(LEDConstants.kSolidWhite), m_statusLED);
        hasTargetTrigger
                .and(isClimbing.negate())
                .and(newTargetTrigger.negate())
                .and(readyToShootTrigger.negate())
                .whenActive(() -> m_statusLED.setAnimation(LEDConstants.kBlinkYellow), m_statusLED);
        isClimbing.whenActive(
                () -> m_statusLED.setAnimation(LEDConstants.kSolidYellow), m_statusLED);
    }

    public Command accelerateAndShoot() {
        return new InstantCommand(() -> currentState.shooterState = ShooterState.SHOOT)
                .alongWith(
                        flywheelCommands.variableVelocity(this::getAimedFlywheelSurfaceVel),
                        columnTopCommands
                                .getRunOutwards()
                                .withTimeout(0.2)
                                .andThen(
                                        columnTopCommands.getGoVariableVelocity(
                                                this::getAimedKickerVelocity)),
                        feederCommands
                                .runColumnBottomOut()
                                .withTimeout(0.2)
                                .andThen(
                                        waitForShooter(),
                                        feederCommands.retractStopper(),
                                        feederCommands
                                                .feedIn(() -> 2.5, () -> 2.5)
                                                .withInterrupt(this::getNotReadyToShoot),
                                        feederCommands.extendStopper(),
                                        waitForShooter(),
                                        feederCommands.retractStopper(),
                                        feederCommands
                                                .feedIn(() -> 2.5, () -> 2.5)
                                                .withInterrupt(this::getNotReadyToShoot),
                                        feederCommands.extendStopper()));
    }

    public Command autoClimbSequnce() {
        return new ConditionalCommand(
                        climberCommands.toNextRung(),
                        new InstantCommand(() -> currentState.climberState = ClimberState.CLIMB)
                                .andThen(
                                        turretCommands
                                                .motionMagic(180)
                                                .andThen(climberCommands.deploy())),
                        this::isClimbing)
                .alongWith(new ScheduleCommand(flywheelCommands.stop()));
    }

    public Command spinupUpToDistance(double maxDistance) {
        return flywheelCommands.variableVelocity(
                () ->
                        FlywheelConstants.getFlywheelRPM(
                                Math.min(getDistanceToTarget(), maxDistance)));
    }

    public Command batterSpinup() {
        return flywheelCommands.variableVelocity(() -> 0.304);
    }

    public Command extendClimberIfClimbing() {
        return new ConditionalCommand(
                climberCommands.setAngled(), new InstantCommand(), this::isClimbing);
    }

    public Command retractClimberIfClimbing() {
        return new ConditionalCommand(
                climberCommands.setStraight(), new InstantCommand(), this::isClimbing);
    }

    public Command aimTurret() {
        return new ConditionalCommand(
                new InstantCommand(),
                new AimTurret(
                        m_turret,
                        this::getAimingParameters,
                        deck.getTracker()::getMeasuredVelocity),
                this::isClimbing);
    }

    public Command climberManualControl(DoubleSupplier percentOut) {
        return new ConditionalCommand(
                climberCommands.openLoopControl(percentOut),
                new InstantCommand(),
                this::isClimbing);
    }

    public Command waitForShooter() {
        return new WaitUntilCommand(this::getReadyToShoot);
    }

    public Command deployAndRunIntake(DoubleSupplier surfaceVelocity) {
        return intakeCommands
                .deploy()
                .andThen(
                        new ConditionalCommand(
                                intakeCommands.runOpenLoop(0.3),
                                intakeCommands.runClosedLoop(surfaceVelocity),
                                () -> this.currentState.shooterState == ShooterState.SHOOT));
    }

    public Command runFeeder(DoubleSupplier surfaceVelocity) {
        return feederCommands
                .extendStopper()
                .andThen(feederCommands.feedIn(surfaceVelocity, surfaceVelocity));
    }

    public AimingParameters getAimingParameters() {
        return aimingParameters;
    }

    public void periodic(double timestamp) {
        aimingParameters = deck.getLatestParameters();
        if (aimingParameters != null) {
            flywheelVelocity = FlywheelConstants.getFlywheelRPM(aimingParameters.getRangeMeters());
            kickerVelocity = flywheelVelocity * 0.5;
            hoodAngle = HoodContants.getHoodAngle(aimingParameters.getRangeMeters());
            System.out.println(
                    (Timer.getFPGATimestamp() - aimingParameters.getLastSeenTimestamp()) < 5);
        }
    }

    public boolean isClimbing() {
        return RobotState.CLIMB.equals(currentState);
    }

    public boolean hasTarget() {
        return getAimingParameters() != null;
    }

    public RobotState getRobotState() {
        return currentState;
    }

    public boolean hasNewTarget() {
        var aimingParams = getAimingParameters();
        return (Timer.getFPGATimestamp() - aimingParams.getLastSeenTimestamp()) < 5;
    }

    public double getDistanceToTarget() {
        if (aimingParameters == null) {
            return 0;
        }
        return aimingParameters.getRangeMeters();
    }

    public boolean getReadyToShoot() {
        return Math.abs(m_flywheel.getVelocity() - getAimedFlywheelSurfaceVel()) < 0.1
                && Math.abs(m_columnTop.getVelocity() - getAimedKickerVelocity()) < 0.5
                && Math.abs(m_hood.getAngle() - getAimedHoodAngle()) < 0.1
                && Math.abs(m_flywheel.getVelocity()) > 5
                && Math.abs(m_columnTop.getVelocity()) > 2;
    }

    public boolean getFlywheelReady() {
        return Math.abs(m_flywheel.getVelocity() - getAimedFlywheelSurfaceVel()) < 0.1;
    }

    public boolean getNotReadyToShoot() {
        return !getReadyToShoot();
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

    public boolean isShooting() {
        return currentState.shooterState == ShooterState.SHOOT;
    }

    public double getIntakeSurfaceVel() {
        return 0.0;
    }

    private final FlightDeck deck;
    private final PivotClimber m_climber;
    private final ColumnBottom m_columnBottom;
    private final VerticalRollers m_verticalRollers;
    private final ColumnTop m_columnTop;
    private final Intake m_intake;
    private final Turret m_turret;
    private final Hood m_hood;
    private final Flywheel m_flywheel;
    private final Ballstopper m_ballstopper;
    private final StatusLED m_statusLED;

    public final FlywheelCommands flywheelCommands;
    public final HoodCommands hoodCommands;
    public final ClimberCommands climberCommands;
    public final TurretCommands turretCommands;
    public final ColumnTopCommands columnTopCommands;
    public final FeederCommands feederCommands;
    public final IntakeCommands intakeCommands;

    public final Trigger readyToShootTrigger;
    public final Trigger newTargetTrigger;
    public final Trigger hasTargetTrigger;
    public final Trigger isShooting;
    public final Trigger isClimbing;
}
