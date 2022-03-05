// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3647.frc2022.autonomous.AutoCommands;
import team3647.frc2022.autonomous.AutoConstants;
import team3647.frc2022.commands.ArcadeDrive;
import team3647.frc2022.constants.*;
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
import team3647.frc2022.subsystems.VerticalRollers;
import team3647.frc2022.subsystems.vision.VisionController;
import team3647.lib.GroupPrinter;
import team3647.lib.inputs.Joysticks;
import team3647.lib.tracking.FlightDeck;
import team3647.lib.tracking.RobotTracker;
import team3647.lib.vision.MultiTargetTracker;
import team3647.lib.vision.PhotonVisionCamera;

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
                m_verticalRollers,
                m_intake,
                m_flywheel,
                m_pivotClimber,
                m_visionController,
                m_turret,
                m_hood,
                m_statusLED);
        // Configure the button bindings
        m_drivetrain.init();
        configureDefaultCommands();
        configureButtonBindings();
        configureSmartDashboardLogging();
        m_hood.resetEncoder();
        HoodContants.kHoodMotor.configAllSettings(HoodContants.kMasterConfig);

        /*m_drivetrain.setOdometry(
                AutoConstants.positionOnTarmacParallel,
                AutoConstants.positionOnTarmacParallel.getRotation());
        runningAutoSequence = autoCommands.getOneTraj();*/

        m_drivetrain.setOdometry(
                AutoConstants.startingStraight, AutoConstants.startingStraight.getRotation());
        runningAutoSequence = autoCommands.getStraightTurn();
    }

    private void configureDefaultCommands() {
        m_drivetrain.setDefaultCommand(
                new ArcadeDrive(
                        m_drivetrain,
                        mainController::getLeftStickY,
                        mainController::getRightStickX,
                        () -> mainController.rightJoyStickPress.get()));
        m_hood.setDefaultCommand(m_superstructure.getHoodAutoAdjustCommand());
        m_flywheel.setDefaultCommand(
                m_superstructure.flywheelCommands.waitToSpinDownThenHold(
                        FlywheelConstants.constantVelocityMpS));
        m_turret.setDefaultCommand(m_superstructure.turretCommands.holdPositionAtCall());
        m_pivotClimber.setDefaultCommand(new RunCommand(m_pivotClimber::end, m_pivotClimber));
    }

    private void configureButtonBindings() {
        // mainController.rightTrigger.whenHeld(m_superstructure.getAutoAimAndShootSensored());
        mainController
                .rightTrigger
                .and(coController.rightTrigger.negate())
                .whileActiveOnce(m_superstructure.getAutoAimAndShootStopper());

        mainController.buttonX.whenPressed(m_superstructure.getAutoClimbSequence());
        mainController.leftBumper.whenHeld(m_superstructure.getClimberManualControl(() -> 0.5));
        mainController.rightBumper.whenHeld(m_superstructure.getClimberManualControl(() -> -0.5));
        mainController.dPadUp.whenHeld(m_superstructure.getRetractClimbManual());
        mainController.dPadDown.whenHeld(m_superstructure.getExtendClimberManual());

        coController.buttonA.whenHeld(
                m_superstructure.getSpinupCommandWithMaxDistance(
                        GlobalConstants.kDistanceFarToGoalCenter));
        coController.buttonB.whenHeld(
                m_superstructure.getSpinupCommandWithMaxDistance(
                        GlobalConstants.kDistanceTarmacToGoalCenter));
        coController.buttonY.whenHeld(
                m_superstructure
                        .turretCommands
                        .getTurretMotionMagic(0)
                        .alongWith(m_superstructure.getBatterSpinupCommand()));
        coController.leftBumper.whenHeld(m_superstructure.getAimTurretCommand());

        coController
                .rightTrigger
                .and(mainController.rightTrigger.negate())
                .whileActiveOnce(m_superstructure.getBallstopIntakeCommand(() -> 0.3));
    }

    private void configureSmartDashboardLogging() {
        m_printer.addDouble("Shooter velocity", m_flywheel::getVelocity);
        m_printer.addDouble("Needed velocity", m_superstructure::getAimedFlywheelSurfaceVel);
        m_printer.addDouble("kicker Needed velocity", m_superstructure::getAimedKickerVelocity);
        m_printer.addBoolean("Ready to Shoot", m_superstructure::getReadyToShoot);
        m_printer.addDouble("Kicker Velocity", m_columnTop::getVelocity);
        m_printer.addDouble("Shooter current", m_flywheel::getMasterCurrent);
        m_printer.addDouble("Kicker current", m_columnTop::getMasterCurrent);
        m_printer.addDouble("Hood Position", m_hood::getPosition);
        m_printer.addDouble("Turret Angle", m_turret::getAngle);
        m_printer.addDouble("Turret Pos", m_turret::getPosition);
        m_printer.addBoolean("Top sensor", m_columnTop::getTopBannerValue);
        m_printer.addBoolean("mid sensor", m_columnBottom::getMiddleBannerValue);
        m_printer.addBoolean("low sensor", m_columnBottom::getBottomBannerValue);
        m_printer.addDouble("Left Climber", m_pivotClimber::getLeftPosition);
        m_printer.addDouble("Right Climber", m_pivotClimber::getRightPosition);
        m_printer.addBoolean("Right stick", () -> mainController.rightJoyStickPress.get());
        m_printer.addDouble("Right Demand", m_drivetrain::getRightDemand);
        m_printer.addDouble("Left Demand", m_drivetrain::getLeftDemand);
        m_printer.addDouble("x position", m_drivetrain::getDrivetrainXMeters);
        m_printer.addDouble("y position", m_drivetrain::getDrivetrainYMeters);

        // m_printer.addPose("Vision Pose", this::getVisionPose);
        m_printer.addPose("Drivetrain Pose", m_drivetrain::getPose);
        SmartDashboard.putNumber("Shooter Speed", 0.0);
        SmartDashboard.putNumber("Hood angle", 16.0);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }

    public double getShooterSpeed() {
        return SmartDashboard.getNumber("Shooter Speed", 0.0);
    }

    public double getHoodDegree() {
        return SmartDashboard.getNumber("Hood angle", 16.0);
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
                    ColumnBottomConstants.kBottomBanner,
                    ColumnBottomConstants.kMiddleBanner,
                    ColumnBottomConstants.kNativeVelToSurfaceMpS,
                    ColumnBottomConstants.kPosConverstion,
                    ColumnBottomConstants.kNominalVoltage,
                    GlobalConstants.kDt,
                    ColumnBottomConstants.kFeedForward);

    final Ballstopper m_ballstopper = new Ballstopper(ColumnBottomConstants.kBallstopperPiston);

    final VerticalRollers m_verticalRollers =
            new VerticalRollers(
                    VerticalRollersConstants.kVerticalRollersMotor,
                    VerticalRollersConstants.kNativeVelToSurfaceMpS,
                    VerticalRollersConstants.kPosConverstion,
                    VerticalRollersConstants.kNominalVoltage,
                    GlobalConstants.kDt,
                    VerticalRollersConstants.kFeedForward);

    final ColumnTop m_columnTop =
            new ColumnTop(
                    ColumnTopConstants.kColumnMotor,
                    ColumnTopConstants.kTopBanner,
                    ColumnTopConstants.kNativeVelToSurfaceMpS,
                    ColumnTopConstants.kPosConverstion,
                    ColumnTopConstants.kNominalVoltage,
                    GlobalConstants.kDt,
                    ColumnTopConstants.kFeedForward);

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
                    new PhotonVisionCamera("gloworm", 0.06, VisionConstants.limelightConstants),
                    VisionConstants.kCenterGoalTargetConstants,
                    m_flightDeck::addVisionObservation);
    final Superstructure m_superstructure =
            new Superstructure(
                    m_flightDeck,
                    m_pivotClimber,
                    m_columnBottom,
                    m_verticalRollers,
                    m_columnTop,
                    m_intake,
                    m_turret,
                    m_hood,
                    m_flywheel,
                    m_ballstopper,
                    m_statusLED);
    private final Command runningAutoSequence;
    /*private final AutoCommands autoCommands =
    new AutoCommands(
            m_drivetrain,
            DrivetrainConstants.kDriveKinematics,
            m_superstructure.getBallstopIntakeCommand(() -> 0.4),
            m_superstructure.getAimTurretCommand(),
            m_superstructure
                    .turretCommands
                    .getTurretMotionMagic(0)
                    .alongWith(m_superstructure.getBatterSpinupCommand()),
            m_superstructure.getShootCommand());*/
    private final AutoCommands autoCommands =
            new AutoCommands(
                    m_drivetrain,
                    DrivetrainConstants.kDriveKinematics,
                    new WaitCommand(2),
                    new WaitCommand(2),
                    new WaitCommand(2),
                    new WaitCommand(2));
}
