// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.LinkedList;
import java.util.List;
import team3647.frc2022.autonomous.AutoCommands;
import team3647.frc2022.autonomous.AutoConstants;
import team3647.frc2022.commands.ArcadeDrive;
import team3647.frc2022.constants.*;
import team3647.frc2022.states.ShooterState;
import team3647.frc2022.states.TurretState;
import team3647.frc2022.subsystems.Ballstopper;
import team3647.frc2022.subsystems.ClimberArm;
import team3647.frc2022.subsystems.ColumnBottom;
import team3647.frc2022.subsystems.ColumnTop;
import team3647.frc2022.subsystems.Drivetrain;
import team3647.frc2022.subsystems.Flywheel;
import team3647.frc2022.subsystems.Hood;
import team3647.frc2022.subsystems.Intake;
import team3647.frc2022.subsystems.PivotClimber;
import team3647.frc2022.subsystems.StatusLED;
import team3647.frc2022.subsystems.Superstructure;
import team3647.frc2022.subsystems.Turret;
import team3647.frc2022.subsystems.vision.VisionController;
import team3647.lib.GroupPrinter;
import team3647.lib.NetworkColorSensor;
import team3647.lib.inputs.Joysticks;
import team3647.lib.tracking.FlightDeck;
import team3647.lib.tracking.RobotTracker;
import team3647.lib.vision.Limelight;
import team3647.lib.vision.MultiTargetTracker;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        pdp.clearStickyFaults();

        m_commandScheduler.registerSubsystem(
                m_drivetrain,
                m_printer,
                m_columnTop,
                m_columnBottom,
                m_intake,
                m_flywheel,
                m_pivotClimber,
                m_visionController,
                m_ballstopper,
                m_turret,
                m_hood,
                m_statusLED);
        // Configure the button bindings
        // m_drivetrain.init();
        configureDefaultCommands();
        configureButtonBindings();
        configureSmartDashboardLogging();
        m_hood.resetEncoder();
        HoodContants.kHoodMotor.configAllSettings(HoodContants.kMasterConfig);

        // chooseAuto();
        m_drivetrain.setOdometry(startPosition, startPosition.getRotation());
    }

    private void configureDefaultCommands() {
        m_drivetrain.setDefaultCommand(
                new ArcadeDrive(
                        m_drivetrain,
                        mainController::getLeftStickY,
                        mainController::getRightStickX,
                        () -> mainController.rightJoyStickPress.get()));
        m_hood.setDefaultCommand(
                m_superstructure.hoodCommands.autoAdjustAngle(m_superstructure::getAimedHoodAngle));
        m_ballstopper.setDefaultCommand(
                m_superstructure.feederCommands.extendStopper().perpetually());
        m_flywheel.setDefaultCommand(
                new InstantCommand(
                                () ->
                                        m_superstructure.currentState.shooterState =
                                                ShooterState.IDLE)
                        .andThen(m_superstructure.flywheelCommands.waitToSpinDownThenHold(0)));
        m_turret.setDefaultCommand(
                new InstantCommand(
                                () ->
                                        m_superstructure.currentState.turretState =
                                                TurretState.HOLD_POSITION)
                        .andThen(m_superstructure.turretCommands.holdPositionAtCall()));
        // m_turret.setDefaultCommand(
        //         new RunCommand(
        //                 () -> m_turret.setOpenloop(coController.getRightStickX()), m_turret));
        m_pivotClimber.setDefaultCommand(new RunCommand(m_pivotClimber::end, m_pivotClimber));
        m_intake.setDefaultCommand(
                m_superstructure.intakeInThenManual(coController::getLeftStickY));
        m_columnBottom.setDefaultCommand(
                m_superstructure.feederInThenManual(coController::getLeftStickY));
        m_superstructure
                .wrongBallDetected
                .and(new Trigger(m_columnTop::getTopBannerValue))
                .whenActive(m_superstructure.rejectBallBottom(), false);
        m_superstructure
                .wrongBallDetected
                .and(new Trigger(() -> !m_columnTop.getTopBannerValue()))
                .whenActive(m_superstructure.rejectBallTop(), false);
    }

    private void configureButtonBindings() {
        // mainController.leftTrigger.whileActiveContinuous(
        //         m_superstructure
        //                 .feederCommands
        //                 .retractStopper()
        //                 .andThen(
        //                         m_superstructure
        //                                 .flywheelCommands
        //                                 .variableVelocity(this::getShooterSpeed)
        //                                 .alongWith(
        //                                         m_superstructure.columnTopCommands
        //                                                 .getGoVariableVelocity(
        //                                                         () -> this.getShooterSpeed() *
        // 0.5),
        //                                         m_superstructure.feederCommands.feedIn(
        //                                                 () -> 1.5))));
        // mainController
        //         .leftTrigger
        //         .whileActiveOnce(m_superstructure.batterAccelerateAndShoot())
        //         .whileActiveOnce(m_superstructure.turretCommands.motionMagic(-180).perpetually())
        //         .whileActiveOnce(
        //                 m_superstructure
        //                         .hoodCommands
        //                         .motionMagic(HoodContants.kBatterAngle)
        //                         .perpetually());
        mainController.leftTrigger.whileActiveOnce(
                m_superstructure.flywheelCommands.variableVelocity(this::getShooterSpeed));
        mainController
                .rightTrigger
                .whileActiveOnce(m_superstructure.autoAccelerateAndShoot())
                .whileActiveOnce(m_superstructure.aimTurret())
                .whileActiveOnce(m_superstructure.intakeCommands.runOpenLoop(.6).withTimeout(0.5));

        mainController.buttonX.whenPressed(m_superstructure.autoClimbSequnce());
        mainController.leftBumper.whenHeld(m_superstructure.climberManualControl(() -> 0.5));
        mainController.rightBumper.whenHeld(m_superstructure.climberManualControl(() -> -0.6));
        mainController.dPadUp.whenHeld(m_superstructure.retractClimberIfClimbing());
        mainController.dPadDown.whenHeld(m_superstructure.extendClimberIfClimbing());

        coController
                .buttonY
                .whileActiveOnce(m_superstructure.batterAccelerateAndShoot())
                .whileActiveOnce(m_superstructure.turretCommands.motionMagic(-180).perpetually())
                .whileActiveOnce(
                        m_superstructure
                                .hoodCommands
                                .motionMagic(HoodContants.kBatterAngle)
                                .perpetually());
        coController
                .buttonA
                .whileActiveOnce(m_superstructure.lowAccelerateAndShoot())
                .whileActiveOnce(m_superstructure.turretCommands.motionMagic(0).perpetually())
                .whileActiveOnce(m_superstructure.intakeCommands.runOpenLoop(.6).withTimeout(0.5))
                .whileActiveOnce(
                        m_superstructure
                                .hoodCommands
                                .motionMagic(HoodContants.kLowGoalAngle)
                                .perpetually());

        coController.leftBumper.whileActiveOnce(m_superstructure.aimTurret());

        coController
                .rightTrigger
                .whileActiveOnce(
                        m_superstructure.deployAndRunIntake(this::calculateIntakeSurfaceSpeed))
                .and(m_superstructure.isShooting.negate())
                .whileActiveOnce(
                        m_superstructure.feederWithSensor(this::calculateIntakeSurfaceSpeed));
        coController.dPadDown.whileActiveOnce(
                m_superstructure.hoodCommands.autoAdjustAngle(this::getHoodDegree));

        coController.dPadUp.whileActiveOnce(m_superstructure.clearFeederFlywheel());

        // mainController.buttonA.whenActive(
        //         () -> {
        //             this.autoCommand = autoCommands.getLowFive();
        //             this.startPosition = AutoConstants.positionOnTarmacParallel;
        //         });
        // mainController.buttonY.whenActive(
        //         () -> {
        //             this.autoCommand = autoCommands.getHighTwo();
        //             this.startPosition = AutoConstants.upperPositionOnTarmac;
        //         });
    }

    private void configureSmartDashboardLogging() {
        m_printer.addDouble("Shooter velocity", m_flywheel::getVelocity);
        m_printer.addDouble("Needed velocity", m_superstructure::getAimedFlywheelSurfaceVel);
        m_printer.addDouble("kicker Needed velocity", m_superstructure::getAimedKickerVelocity);
        m_printer.addBoolean("Ready to Shoot", m_superstructure::readyToAutoShoot);
        m_printer.addDouble("Kicker Velocity", m_columnTop::getVelocity);
        m_printer.addDouble("Shooter current", m_flywheel::getMasterCurrent);
        m_printer.addDouble("Kicker current", m_columnTop::getMasterCurrent);
        m_printer.addDouble("Hood Position", m_hood::getPosition);
        m_printer.addDouble("Turret Angle", m_turret::getAngle);
        m_printer.addDouble("Turret Pos", m_turret::getPosition);
        m_printer.addDouble("Left Climber", m_pivotClimber::getLeftPosition);
        m_printer.addDouble("Right Climber", m_pivotClimber::getRightPosition);
        m_printer.addBoolean("Right stick", () -> mainController.rightJoyStickPress.get());
        m_printer.addDouble("Target Range", m_superstructure::getDistanceToTarget);
        m_printer.addString("Color", m_columnBottom.getColorSensor()::getColorAsString);
        m_printer.addBoolean("Read Color", m_columnBottom.getColorSensor()::isReadColor);
        m_printer.addBoolean("Top Sensor", m_columnTop::getTopBannerValue);
        m_printer.addPose(
                "Vision Pose",
                () -> {
                    var aimingParams = m_superstructure.getAimingParameters();
                    if (aimingParams == null) {
                        return new Pose2d();
                    }
                    return aimingParams.getFieldToGoal();
                });
        m_printer.addPose(
                "Turret Pose",
                () -> {
                    var turretPose =
                            m_flightDeck.getTracker().getFieldToTurret(Timer.getFPGATimestamp());
                    if (turretPose == null) {
                        return new Pose2d();
                    }
                    return turretPose;
                });
        m_printer.addPose(
                "Camera Pose",
                () -> {
                    var camPose =
                            m_flightDeck.getFieldToCamera(
                                    TurretConstants.kTurretToCamFixedTransform);
                    if (camPose == null) {
                        return new Pose2d();
                    }
                    return camPose;
                });
        m_printer.addPose("Drivetrain Pose", m_drivetrain::getPose);
        SmartDashboard.putNumber("Shooter Speed", 0.0);
        SmartDashboard.putNumber("Shooter Speed Offset", 0.0);
        SmartDashboard.putNumber("Hood angle", 15.0);
        SmartDashboard.putNumber("extra latency", 0.0);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoCommand;
    }

    public double getShooterSpeed() {
        return SmartDashboard.getNumber("Shooter Speed", 0.0);
    }

    public double getHoodDegree() {
        return SmartDashboard.getNumber("Hood angle", 16.0);
    }

    public double calculateIntakeSurfaceSpeed() {
        double leftVel = Math.abs(m_drivetrain.getLeftVelocity());
        double rightVel = Math.abs(m_drivetrain.getRightVelocity());
        double max = Math.max(leftVel, rightVel) * 2;
        return max < 3 ? 3 : max;
    }

    private final Translation2d toCenter = new Translation2d(5, 5);

    public void updateTapeTranslations(List<Translation2d> translations) {
        List<Pose2d> poses = new LinkedList<>();
        translations.stream()
                .map((translation) -> new Pose2d(translation, new Rotation2d()))
                .map(m_flightDeck::getInFieldCoordinatesFromCamera)
                .forEach(poses::add);
        m_printer.getField().getObject("Tapes").setPoses(poses);
    }

    private final CommandScheduler m_commandScheduler = CommandScheduler.getInstance();
    private final PowerDistribution pdp = new PowerDistribution(1, ModuleType.kRev);

    private final Joysticks mainController = new Joysticks(0);
    private final Joysticks coController = new Joysticks(1);

    private final GroupPrinter m_printer = GroupPrinter.getInstance();

    final Drivetrain m_drivetrain =
            new Drivetrain(
                    DrivetrainConstants.kLeftMaster,
                    DrivetrainConstants.kRightMaster,
                    DrivetrainConstants.kLeftSlave,
                    DrivetrainConstants.kRightSlave,
                    DrivetrainConstants.kPigeonIMU,
                    DrivetrainConstants.kFeedforward,
                    DrivetrainConstants.kPoseEstimator,
                    DrivetrainConstants.kFalconVelocityToMpS,
                    DrivetrainConstants.kFalconTicksToMeters,
                    DrivetrainConstants.kNominalVoltage,
                    GlobalConstants.kDt);

    final Flywheel m_flywheel =
            new Flywheel(
                    FlywheelConstants.kMaster,
                    FlywheelConstants.kNativeVelToSurfaceMpS,
                    0,
                    FlywheelConstants.kNominalVoltage,
                    GlobalConstants.kDt,
                    FlywheelConstants.kFollower,
                    FlywheelConstants.kFeedForward);

    final Intake m_intake =
            new Intake(
                    IntakeConstants.kIntakeMotor,
                    IntakeConstants.nativeVelToSurfaceMpS,
                    0,
                    IntakeConstants.kNominalVoltage,
                    GlobalConstants.kDt,
                    IntakeConstants.kFeedForward,
                    IntakeConstants.kPistons);

    final ColumnBottom m_columnBottom =
            new ColumnBottom(
                    ColumnBottomConstants.kColumnMotor,
                    ColumnBottomConstants.kNativeVelToSurfaceMpS,
                    ColumnBottomConstants.kPosConverstion,
                    ColumnBottomConstants.kNominalVoltage,
                    GlobalConstants.kDt,
                    ColumnBottomConstants.kFeedForward,
                    new NetworkColorSensor(
                            ColorsensorConstants.kProximityEntry,
                            ColorsensorConstants.kColorEntry,
                            ColorsensorConstants.kMaxReadDistance));

    final Ballstopper m_ballstopper = new Ballstopper(ColumnBottomConstants.kBallstopperPiston);

    final ColumnTop m_columnTop =
            new ColumnTop(
                    ColumnTopConstants.kColumnMotor,
                    ColumnTopConstants.kTopBanner,
                    ColumnTopConstants.kNativeVelToSurfaceMpS,
                    ColumnTopConstants.kPosConverstion,
                    ColumnTopConstants.kNominalVoltage,
                    GlobalConstants.kDt,
                    ColumnTopConstants.kFeedForward,
                    ColumnTopConstants.kTopBanner);

    private final ClimberArm m_leftArm =
            new ClimberArm(
                    ClimberConstants.kLeftMotor,
                    ClimberConstants.kNativeVelToMpS,
                    ClimberConstants.kPosConverstion,
                    ClimberConstants.kNominalVoltage,
                    GlobalConstants.kDt);

    private final ClimberArm m_rightArm =
            new ClimberArm(
                    ClimberConstants.kRightMotor,
                    ClimberConstants.kNativeVelToMpS,
                    ClimberConstants.kPosConverstion,
                    ClimberConstants.kNominalVoltage,
                    GlobalConstants.kDt);

    final PivotClimber m_pivotClimber =
            new PivotClimber(
                    m_leftArm,
                    m_rightArm,
                    ClimberConstants.kPivotPistons,
                    ClimberConstants.kMaxLengthAngled,
                    ClimberConstants.kMaxLengthStraight,
                    ClimberConstants.kVoltageToHoldRobot);

    final Hood m_hood =
            new Hood(
                    HoodContants.kHoodMotor,
                    HoodContants.kFalconVelocityToDegpS,
                    HoodContants.kFalconPositionToDegrees,
                    HoodContants.kNominalVoltage,
                    HoodContants.kS,
                    HoodContants.kCos,
                    GlobalConstants.kDt,
                    HoodContants.kMinDegree,
                    HoodContants.kMaxDegree,
                    HoodContants.kPosThersholdDeg);

    final Turret m_turret =
            new Turret(
                    TurretConstants.kTurretMotor,
                    TurretConstants.kFalconVelocityToDegpS,
                    TurretConstants.kFalconPositionToDegrees,
                    TurretConstants.kNominalVoltage,
                    GlobalConstants.kDt,
                    TurretConstants.kS,
                    TurretConstants.kMaxDegree,
                    TurretConstants.kMinDegree,
                    TurretConstants.kStartingAngle,
                    TurretConstants.kTurretProfile,
                    TurretConstants.kFeedForwards);

    final StatusLED m_statusLED = new StatusLED(LEDConstants.kCANdle);

    final FlightDeck m_flightDeck =
            new FlightDeck(
                    new RobotTracker(
                            1.0,
                            TurretConstants.kRobotToTurretFixed,
                            m_drivetrain::getPose,
                            m_drivetrain::getTimestamp,
                            m_turret::getRotation,
                            m_turret::getTimestamp),
                    new MultiTargetTracker(),
                    TurretConstants.kTurretToCamFixed);

    final VisionController m_visionController =
            new VisionController(
                    new Limelight("10.36.47.15", 0.018, VisionConstants.limelightConstants),
                    VisionConstants.kCenterGoalTargetConstants,
                    m_flightDeck::addVisionObservation,
                    this::updateTapeTranslations);

    private final Compressor compressor = new Compressor(GlobalConstants.kPCMType);

    final Superstructure m_superstructure =
            new Superstructure(
                    m_flightDeck,
                    m_pivotClimber,
                    m_columnBottom,
                    m_columnTop,
                    m_intake,
                    m_turret,
                    m_hood,
                    m_flywheel,
                    m_ballstopper,
                    compressor,
                    m_statusLED,
                    m_drivetrain::isStopped);

    private final AutoCommands autoCommands =
            new AutoCommands(m_drivetrain, DrivetrainConstants.kDriveKinematics, m_superstructure);
    private Pose2d startPosition = AutoConstants.positionOnTarmacUpper;
    private Command autoCommand = autoCommands.getHighTwoStay();
    public Auto currentAuto = Auto.HIGH_TWO;

    public enum Auto {
        LOW_FIVE(0),
        HIGH_TWO_ONE(1),
        HIGH_TWO(2),
        HIGH_TWO_TWO(3);
        int index;

        Auto(int index) {
            this.index = index;
        }
    }

    public void chooseAuto() {
        switch (currentAuto) {
            case LOW_FIVE:
                startPosition = AutoConstants.positionOnTarmacParallel;
                autoCommand = autoCommands.lowFiveClean();
                break;
            case HIGH_TWO_ONE:
                startPosition = AutoConstants.positionOnTarmacUpper;
                autoCommand = autoCommands.getHighTwoSendOnetoHangar();
                break;
            case HIGH_TWO:
                startPosition = AutoConstants.positionOnTarmacUpper;
                autoCommand = autoCommands.getHighTwoStay();
                break;
            case HIGH_TWO_TWO:
                startPosition = AutoConstants.positionOnTarmacUpper;
                autoCommand = autoCommands.getHighTwoSendTwotoHangar();
                break;
            default:
                startPosition = AutoConstants.positionOnTarmacParallel;
                autoCommand = autoCommands.lowFiveClean();
                break;
        }
    }
}
