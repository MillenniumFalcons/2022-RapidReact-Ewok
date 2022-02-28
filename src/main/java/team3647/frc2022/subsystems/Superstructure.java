package team3647.frc2022.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    public final Trigger oldTargetTrigger;

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
        this.aimedState = RobotState.STOPPED;

        flywheelCommands = new FlywheelCommands(m_flywheel);
        hoodCommands = new HoodCommands(m_hood);
        climberCommands = new ClimberCommands(m_climber);
        columnTopCommands = new ColumnTopCommands(m_columnTop);
        feederCommands =
                new FeederCommands(m_columnBottom, m_columnTop, m_verticalRollers, m_ballstopper);
        intakeCommands = new IntakeCommands(m_intake);
        turretCommands = new TurretCommands(m_turret);
        readyToShootTrigger = new Trigger(this::getReadyToShoot);
        hasTargetTrigger = new Trigger(this::hasTarget);
        newTargetTrigger = new Trigger(this::hasNewTarget);
        oldTargetTrigger = new Trigger(() -> !this.hasNewTarget());
        hasTargetTrigger
                .negate()
                .whileActiveContinuous(
                        new InstantCommand(
                                () -> m_statusLED.setAnimation(LEDConstants.kBlinkRed),
                                m_statusLED));
        //         .whileActiveOnce(new PrintCommand("Lost Target"));
        // // oldTargetTrigger
        // //         .and(readyToShootTrigger.negate())
        // //         .whileActiveOnce(
        // //                 new InstantCommand(
        // //                         () -> statusLEDs.setAnimation(LEDConstants.kBlinkYellow)));
        hasTargetTrigger
                .and(newTargetTrigger)
                .and(readyToShootTrigger)
                .whileActiveContinuous(
                        new InstantCommand(
                                () -> m_statusLED.setAnimation(LEDConstants.kBlinkGreen),
                                m_statusLED));
        hasTargetTrigger
                .and(newTargetTrigger)
                .and(readyToShootTrigger.negate())
                .whileActiveContinuous(
                        new InstantCommand(
                                () -> m_statusLED.setAnimation(LEDConstants.kBlinkWhite),
                                m_statusLED));
        //         .whileActiveOnce(new PrintCommand("Has Target no velocity"));
        // oldTargetTrigger
        //         .and(readyToShootTrigger)
        //         .whileActiveOnce(new InstantCommand(() -> statusLEDs.setColor(Color.YELLOW, 0)));
    }

    public Command getShootCommandWithStopper() {
        return flywheelCommands
                .getGoVariableVelocity(this::getAimedFlywheelSurfaceVel)
                .alongWith(columnTopCommands.getGoVariableVelocity(this::getAimedKickerVelocity))
                .alongWith(
                        new InstantCommand()
                                .andThen(
                                        getWaitForShooter(),
                                        feederCommands.getRetractBallstopper(),
                                        feederCommands.getFeedInwardsUntil(
                                                this::getNotReadyToShoot),
                                        feederCommands.getExtendBallstopper(),
                                        getWaitForShooter()
                                                .deadlineWith(feederCommands.getFeedInwards()),
                                        feederCommands.getRetractBallstopper(),
                                        feederCommands.getFeedInwardsUntil(
                                                this::getNotReadyToShoot),
                                        feederCommands.getExtendBallstopper()));
    }

    public Command getShootCommand() {
        return flywheelCommands
                .getGoVariableVelocity(this::getAimedFlywheelSurfaceVel)
                .alongWith(
                        columnTopCommands.getGoVariableVelocity(this::getAimedKickerVelocity),
                        getWaitForShooter()
                                .andThen(
                                        feederCommands
                                                .getFeedInwardsUntil(this::getNotReadyToShoot)
                                                .andThen(getWaitForShooter()))
                                .andThen(
                                        feederCommands.getFeedInwardsUntil(
                                                this::getNotReadyToShoot)));
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

    public Command getSpinupCommandWithMaxDistance(double maxDistance) {
        return flywheelCommands.getGoVariableVelocity(
                () ->
                        FlywheelConstants.getFlywheelRPM(
                                Math.min(getDistanceToTarget(), maxDistance)));
    }

    public Command getBatterSpinupCommand() {
        // distance from bumper to center of turret from the intake side (meters)
        return flywheelCommands.getGoVariableVelocity(() -> 0.304);
    }

    public Command getAutoAimAndShootSensored() {
        return getAimTurretCommand().alongWith(getShootCommand());
    }

    public Command getAutoAimAndShootStopper() {
        return getAimTurretCommand().alongWith(getShootCommandWithStopper());
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
        return new WaitUntilCommand(this::getReadyToShoot);
    }

    public Command getBallstopIntakeCommand(DoubleSupplier output) {
        return new InstantCommand(m_ballstopper::extend)
                .andThen(
                        intakeCommands
                                .getIntakeSequnce(intakeCommands.getRunIntakeOpenloop(output))
                                .alongWith(feederCommands.getFeedInwards()));
    }

    // public Command getBallStopperIntakeCommand(DoubleSupplier output) {
    //     return intakeCommands
    //             .getIntakeSequnce(intakeCommands.getRunIntakeOpenloop(output))
    //             .alongWith(
    //                     new ConditionalCommand(
    //                             new RunCommand(m_ballstopper::extend, m_ballstopper),
    //                             feederCommands.getFeedInwardsUntil(
    //                                     m_columnBottom::getBottomBannerValue),
    //                             m_columnBottom::getBottomBannerValue));
    // }

    public Command getIntakeHoldCommand(DoubleSupplier output) {
        return intakeCommands
                .getIntakeSequnce(intakeCommands.getRunIntakeOpenloop(output))
                .alongWith(
                        new ConditionalCommand(
                                feederCommands.getBottomCheckFeed(
                                        m_columnTop::getTopBannerValue,
                                        m_columnBottom::getMiddleBannerValue),
                                feederCommands
                                        .getFeedInwards()
                                        .alongWith(
                                                columnTopCommands
                                                        .getRunInwards()
                                                        .withInterrupt(
                                                                m_columnTop::getTopBannerValue)),
                                m_columnTop::getTopBannerValue));
    }

    public Command getIntakeReleaseCommand() {
        return feederCommands
                .getFeedOutwardsUntil(m_columnTop::getNotTopBannerValue)
                .alongWith(columnTopCommands.getRunOutwardsUntil(m_columnTop::getNotTopBannerValue))
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

    public boolean hasTarget() {
        return getAimingParameters() != null;
    }

    public boolean hasNewTarget() {
        var aimingParams = getAimingParameters();
        return aimingParams != null;
        // return (aimingParams.getLastSeenTimestamp() - Timer.getFPGATimestamp()) < 5;
    }

    public double getDistanceToTarget() {
        if (aimingParameters == null) {
            return 0;
        }
        return aimingParameters.getRangeMeters();
    }

    public boolean getReadyToShoot() {
        return m_flywheel.getVelocity() >= getAimedFlywheelSurfaceVel() - 0.1
                && m_columnTop.getVelocity() > getAimedKickerVelocity() - 2
                && Math.abs(m_flywheel.getVelocity()) > 5
                && Math.abs(m_columnTop.getVelocity()) > 2;
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
}
