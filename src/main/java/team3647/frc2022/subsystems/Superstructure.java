package team3647.frc2022.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.HashSet;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import team3647.frc2022.commands.ClimberCommands;
import team3647.frc2022.commands.ColumnTopCommands;
import team3647.frc2022.commands.FeederCommands;
import team3647.frc2022.commands.FlywheelCommands;
import team3647.frc2022.commands.HoodCommands;
import team3647.frc2022.commands.IntakeCommands;
import team3647.frc2022.commands.turret.AimTurret;
import team3647.frc2022.commands.turret.TurretCommands;
import team3647.frc2022.constants.ColumnTopConstants;
import team3647.frc2022.constants.FlywheelConstants;
import team3647.frc2022.constants.HoodContants;
import team3647.frc2022.constants.LEDConstants;
import team3647.frc2022.states.ClimberState;
import team3647.frc2022.states.RobotState;
import team3647.frc2022.states.ShooterState;
import team3647.frc2022.states.TurretState;
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
        feederCommands = new FeederCommands(m_columnBottom, m_columnTop, m_ballstopper);
        intakeCommands = new IntakeCommands(m_intake);
        turretCommands = new TurretCommands(m_turret);
        isClimbing = new Trigger(this::isClimbing);
        hasTargetTrigger = new Trigger(this::hasTarget);
        flywheelOnlyReady = new Trigger(() -> getFlywheelReady(this::getAimedFlywheelSurfaceVel));
        newTargetTrigger = new Trigger(this::hasNewTarget);
        isShooting = new Trigger(this::isShooting);
        isAiming = new Trigger(this::isAiming);
        fullyReadyToShoot = new Trigger(this::getReadyToAutoShoot);
    }

    public Command autoAccelerateAndShoot() {
        return accelerateAndShoot(
                this::getAimedFlywheelSurfaceVel,
                this::getAimedKickerVelocity,
                this::getReadyToAutoShoot);
    }

    public Command batterAccelerateAndShoot() {
        return new WaitUntilCommand(() -> Math.abs(m_turret.getAngle() + 180) < 1)
                .andThen(
                        accelerateAndShoot(
                                () -> FlywheelConstants.kBatterVelocity,
                                () -> ColumnTopConstants.kBatterVelocity,
                                this::getReadyToBatter));
    }

    public Command lowAccelerateAndShoot() {
        return new WaitUntilCommand(() -> Math.abs(m_turret.getAngle() - 0) < 1)
                .andThen(
                        accelerateAndShoot(
                                () -> FlywheelConstants.kLowGoalVelocity,
                                () -> ColumnTopConstants.kLowGoalVelocity,
                                this::getReadyToLowGoal));
    }

    public Command accelerateAndShoot(
            DoubleSupplier flywhelVelocity,
            DoubleSupplier kickerVelocity,
            BooleanSupplier readyToShoot) {
        return new InstantCommand(() -> currentState.shooterState = ShooterState.SHOOT)
                .alongWith(
                        flywheelCommands.variableVelocity(flywhelVelocity),
                        columnTopCommands
                                .getRunOutwards()
                                .withTimeout(0.2)
                                .andThen(columnTopCommands.getGoVariableVelocity(kickerVelocity)),
                        feederCommands
                                .runColumnBottomOut()
                                .withTimeout(0.2)
                                .andThen(
                                        new WaitUntilCommand(readyToShoot),
                                        feederCommands.retractStopper(),
                                        waitUntilAndTimeout(() -> !readyToShoot.getAsBoolean(), 0.5)
                                                .deadlineWith(
                                                        feederCommands.feedIn(() -> 1.5, () -> 1)),
                                        feederCommands.extendStopper(),
                                        waitUntilAndTimeout(readyToShoot, 0.4),
                                        feederCommands.retractStopper(),
                                        waitUntilAndTimeout(() -> !readyToShoot.getAsBoolean(), 2)
                                                .deadlineWith(
                                                        feederCommands.feedIn(() -> 4, () -> 1)),
                                        feederCommands.extendStopper()));
    }

    public Command autoClimbSequnce() {
        return new ConditionalCommand(
                        climberCommands.toNextRung(),
                        new InstantCommand(
                                        () -> {
                                            currentState.climberState = ClimberState.CLIMB;
                                            currentState.turretState = TurretState.HOLD_POSITION;
                                        })
                                .andThen(
                                        turretCommands
                                                .motionMagic(0)
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
                new InstantCommand(() -> this.currentState.turretState = TurretState.AIM)
                        .andThen(
                                new AimTurret(
                                        m_turret,
                                        this::getAimingParameters,
                                        deck.getTracker()::getMeasuredVelocity)),
                this::isClimbing);
    }

    public Command climberManualControl(DoubleSupplier percentOut) {
        return new ConditionalCommand(
                climberCommands.openLoopControl(percentOut),
                new InstantCommand(),
                this::isClimbing);
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

    public Command waitUntilAndTimeout(BooleanSupplier interrupt, double timeoutSec) {
        Timer m_timer = new Timer();
        return new Command() {
            @Override
            public void initialize() {
                m_timer.reset();
                m_timer.start();
            }

            @Override
            public boolean isFinished() {
                return m_timer.hasElapsed(timeoutSec) && interrupt.getAsBoolean();
            }

            @Override
            public Set<Subsystem> getRequirements() {
                return new HashSet<>();
            }
        };
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
        }
    }

    public boolean isClimbing() {
        return currentState.climberState == ClimberState.CLIMB;
    }

    public boolean hasTarget() {
        return getAimingParameters() != null;
    }

    public RobotState getRobotState() {
        return currentState;
    }

    public boolean hasNewTarget() {
        var aimingParams = getAimingParameters();
        if (aimingParams == null) {
            return false;
        }
        return (Timer.getFPGATimestamp() - aimingParams.getLastSeenTimestamp()) < 5;
    }

    public double getDistanceToTarget() {
        if (aimingParameters == null) {
            return 0;
        }
        return aimingParameters.getRangeMeters();
    }

    public boolean getReadyToShoot(
            DoubleSupplier flywheel, DoubleSupplier kicker, DoubleSupplier hood) {
        return getFlywheelReady(flywheel)
                && Math.abs(m_columnTop.getVelocity() - kicker.getAsDouble()) < 0.5
                && Math.abs(m_hood.getAngle() - hood.getAsDouble()) < 0.1
                && Math.abs(m_flywheel.getVelocity()) > 5
                && Math.abs(m_columnTop.getVelocity()) > 2;
    }

    public boolean getReadyToLowGoal() {
        return getReadyToShoot(
                () -> FlywheelConstants.kLowGoalVelocity,
                () -> ColumnTopConstants.kLowGoalVelocity,
                () -> HoodContants.kLowGoalAngle);
    }

    public boolean getReadyToBatter() {
        return getReadyToShoot(
                () -> FlywheelConstants.kBatterVelocity,
                () -> ColumnTopConstants.kBatterVelocity,
                () -> HoodContants.kBatterAngle);
    }

    public boolean getReadyToAutoShoot() {
        return getReadyToShoot(
                this::getAimedFlywheelSurfaceVel,
                this::getAimedKickerVelocity,
                this::getAimedHoodAngle);
    }

    public boolean getFlywheelReady(DoubleSupplier expectedVelocity) {
        return Math.abs(m_flywheel.getVelocity() - expectedVelocity.getAsDouble()) < 0.1;
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

    public boolean isAiming() {
        return currentState.turretState == TurretState.AIM;
    }

    public double getIntakeSurfaceVel() {
        return 0.0;
    }

    public void configLEDTriggers() {
        var newTargetShooting =
                hasTargetTrigger
                        .and(isClimbing.negate())
                        .and(newTargetTrigger)
                        .and(fullyReadyToShoot)
                        .and(flywheelOnlyReady)
                        .and(isAiming);
        var newTargetReady =
                hasTargetTrigger
                        .and(isClimbing.negate())
                        .and(newTargetTrigger)
                        .and(fullyReadyToShoot.negate())
                        .and(flywheelOnlyReady)
                        .and(isAiming);
        var newTargetNotReady =
                hasTargetTrigger
                        .and(isClimbing.negate())
                        .and(newTargetTrigger)
                        .and(fullyReadyToShoot.negate())
                        .and(flywheelOnlyReady.negate())
                        .and(isAiming.negate());
        var newTargetNotReadyAiming =
                hasTargetTrigger
                        .and(isClimbing.negate())
                        .and(newTargetTrigger)
                        .and(fullyReadyToShoot.negate())
                        .and(flywheelOnlyReady.negate())
                        .and(isAiming);

        var oldTargetShooting =
                hasTargetTrigger
                        .and(isClimbing.negate())
                        .and(newTargetTrigger.negate())
                        .and(fullyReadyToShoot)
                        .and(flywheelOnlyReady)
                        .and(isAiming);
        var oldTargetReady =
                hasTargetTrigger
                        .and(isClimbing.negate())
                        .and(newTargetTrigger.negate())
                        .and(fullyReadyToShoot.negate())
                        .and(flywheelOnlyReady)
                        .and(isAiming);
        var oldTargetNotReadyAiming =
                hasTargetTrigger
                        .and(isClimbing.negate())
                        .and(newTargetTrigger.negate())
                        .and(fullyReadyToShoot.negate())
                        .and(flywheelOnlyReady.negate())
                        .and(isAiming);
        var oldTargetNotReady =
                hasTargetTrigger
                        .and(isClimbing.negate())
                        .and(newTargetTrigger.negate())
                        .and(fullyReadyToShoot.negate())
                        .and(flywheelOnlyReady.negate())
                        .and(isAiming.negate());
        hasTargetTrigger
                .negate()
                .whenActive(() -> m_statusLED.setAnimation(LEDConstants.kBlinkRed), m_statusLED);

        newTargetNotReady.whenActive(
                () -> m_statusLED.setAnimation(LEDConstants.kSolidWhite), m_statusLED);
        newTargetNotReadyAiming.whenActive(
                () -> m_statusLED.setAnimation(LEDConstants.kBlinkWhiteFast), m_statusLED);
        newTargetReady.whenActive(
                () -> m_statusLED.setAnimation(LEDConstants.kBlinkGreen), m_statusLED);
        newTargetShooting.whenActive(
                () -> m_statusLED.setAnimation(LEDConstants.kSolidkGreen), m_statusLED);

        oldTargetNotReady.whenActive(
                () -> m_statusLED.setAnimation(LEDConstants.kSolidYellow), m_statusLED);
        oldTargetNotReadyAiming.whenActive(
                () -> m_statusLED.setAnimation(LEDConstants.kBlinkYellow), m_statusLED);
        oldTargetReady.whenActive(
                () -> m_statusLED.setAnimation(LEDConstants.kBlinkOrange), m_statusLED);
        oldTargetShooting.whenActive(
                () -> m_statusLED.setAnimation(LEDConstants.kSolidOrange), m_statusLED);

        isClimbing.whenActive(() -> m_statusLED.setAnimation(LEDConstants.kSolidPink), m_statusLED);
    }

    private final FlightDeck deck;
    private final PivotClimber m_climber;
    private final ColumnBottom m_columnBottom;
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

    public final Trigger flywheelOnlyReady;
    public final Trigger newTargetTrigger;
    public final Trigger hasTargetTrigger;
    public final Trigger isShooting;
    public final Trigger isClimbing;
    public final Trigger fullyReadyToShoot;
    public final Trigger isAiming;
}
