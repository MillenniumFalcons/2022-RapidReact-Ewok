// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import team3647.frc2022.commands.ArcadeDrive;
import team3647.frc2022.commands.ClimberUpDown;
import team3647.frc2022.commands.IntakeBallTest;
import team3647.frc2022.commands.ShootBall;
import team3647.frc2022.commands.TestHood;
import team3647.frc2022.commands.climber.ClimberDeploy;
import team3647.frc2022.commands.climber.ClimberLength;
import team3647.frc2022.constants.*;
import team3647.frc2022.states.RobotState;
import team3647.frc2022.subsystems.ClimberArm;
import team3647.frc2022.subsystems.ColumnBottom;
import team3647.frc2022.subsystems.ColumnTop;
import team3647.frc2022.subsystems.Drivetrain;
import team3647.frc2022.subsystems.Flywheel;
import team3647.frc2022.subsystems.Hood;
import team3647.frc2022.subsystems.Intake;
import team3647.frc2022.subsystems.PivotClimber;
import team3647.frc2022.subsystems.Superstructure;
import team3647.frc2022.subsystems.Turret;
import team3647.frc2022.subsystems.VerticalRollers;
import team3647.lib.GroupPrinter;
import team3647.lib.inputs.Joysticks;

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
                m_drivetrain, /*m_intake,*/
                m_printer,
                m_columnTop,
                m_columnBottom,
                m_verticalRollers,
                m_intake,
                m_flywheel,
                m_leftArm,
                m_rightArm,
                m_pivotClimber,
                m_hood);
        // Configure the button bindings
        m_drivetrain.init();
        m_drivetrain.setDefaultCommand(
                new ArcadeDrive(
                        m_drivetrain,
                        mainController::getLeftStickY,
                        mainController::getRightStickX));

        // m_columnBottom.setDefaultCommand(new OpenLoop(m_columnBottom,
        // coController::getLeftStickY));
        // // m_verticalRollers.setDefaultCommand(
        // //         new OpenLoop(m_columnBottom, coController::getLeftStickY));
        // m_columnTop.setDefaultCommand(new OpenLoop(m_columnBottom, coController::getLeftStickY));
        m_columnBottom.setDefaultCommand(
                new RunCommand(
                        () -> {
                            m_columnBottom.setOpenloop(coController.getRightStickY());
                        },
                        m_columnBottom));
        m_intake.setDefaultCommand(
                new RunCommand(
                        () -> {
                            m_intake.setOpenloop(-coController.getRightStickY());
                        },
                        m_intake));
        coController.leftTrigger.whenHeld(
                new IntakeBallTest(
                        m_intake,
                        m_columnBottom,
                        m_verticalRollers,
                        coController::getLeftTriggerValue));
        // coController.leftTrigger.whenReleased(new InstantCommand(m_intake::retract, m_intake));
        m_pivotClimber.setDefaultCommand(
                new ClimberUpDown(
                        m_pivotClimber,
                        mainController::getLeftTriggerValue,
                        mainController::getRightTriggerValue));
        configureButtonBindings();
        m_printer.addDouble("Shooter velocity", m_flywheel::getVelocity);
        m_printer.addDouble("LeftStick", coController::getLeftStickY);
        m_printer.addDouble("Kicker Velocity", m_columnTop::getVelocity);
        m_printer.addDouble("Shooter current", m_flywheel::getMasterCurrent);
        m_printer.addDouble("Kicker current", m_columnTop::getMasterCurrent);
        m_printer.addDouble("Hood Position", m_hood::getPosition);
        m_printer.addDouble("Hood native", m_hood::getNativePos);

        SmartDashboard.putNumber("Shooter Speed", 0.0);
        SmartDashboard.putNumber("Hood angle", 16.0);
        m_hood.resetEncoder();
        HoodContants.kHoodMotor.configAllSettings(HoodContants.kMasterConfig);
    }

    private void configureButtonBindings() {
        mainController.buttonX.whenActive(
                new ConditionalCommand(
                        new InstantCommand(m_pivotClimber::setAngled)
                                .andThen(new WaitCommand(0.1))
                                .andThen(
                                        new ClimberLength(
                                                m_pivotClimber, ClimberConstants.kMaxLengthAngled)),
                        new ClimberDeploy(m_pivotClimber, null)
                                .andThen(() -> m_superstructure.setState(RobotState.CLIMB)),
                        m_superstructure::isClimbing));

        mainController.buttonB.whenActive(new InstantCommand(m_pivotClimber::setStraight));

        coController.dPadUp.whenPressed(new TestHood(m_hood, this::getHoodDegree));

        coController.rightTrigger.whenHeld(
                new ShootBall(m_flywheel, m_columnTop, m_columnBottom, this::getShooterSpeed));
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

    private final Drivetrain m_drivetrain =
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

    private final Flywheel m_flywheel =
            new Flywheel(
                    FlywheelConstants.kMaster,
                    FlywheelConstants.kNativeVelToSurfaceMpS,
                    0,
                    FlywheelConstants.kNominalVoltage,
                    GlobalConstants.kDt,
                    FlywheelConstants.kFollower,
                    FlywheelConstants.kFeedForward);

    private final Intake m_intake =
            new Intake(
                    IntakeConstants.kIntakeMotor,
                    IntakeConstants.nativeVelToSurfaceMpS,
                    0,
                    IntakeConstants.kNominalVoltage,
                    GlobalConstants.kDt,
                    IntakeConstants.kFeedForward,
                    IntakeConstants.kPistons);

    private final ColumnBottom m_columnBottom =
            new ColumnBottom(
                    ColumnBottomConstants.kColumnMotor,
                    ColumnBottomConstants.kNativeVelToSurfaceMpS,
                    ColumnBottomConstants.kPosConverstion,
                    ColumnBottomConstants.kNominalVoltage,
                    GlobalConstants.kDt,
                    ColumnBottomConstants.kFeedForward);
    private final VerticalRollers m_verticalRollers =
            new VerticalRollers(
                    VerticalRollersConstants.kVerticalRollersMotor,
                    VerticalRollersConstants.kNativeVelToSurfaceMpS,
                    VerticalRollersConstants.kPosConverstion,
                    VerticalRollersConstants.kNominalVoltage,
                    GlobalConstants.kDt,
                    VerticalRollersConstants.kFeedForward);

    private final ColumnTop m_columnTop =
            new ColumnTop(
                    ColumnTopConstants.kColumnMotor,
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
    private final PivotClimber m_pivotClimber =
            new PivotClimber(
                    m_leftArm,
                    m_rightArm,
                    ClimberConstants.kPivotPistons,
                    ClimberConstants.kMaxLengthAngled,
                    ClimberConstants.kMaxLengthStraight,
                    ClimberConstants.kVoltageToHoldRobot);

    private final Hood m_hood =
            new Hood(
                    HoodContants.kHoodMotor,
                    HoodContants.kFalconVelocityToDegpS,
                    HoodContants.kFalconPositionToDegrees,
                    HoodContants.kNominalVoltage,
                    GlobalConstants.kDt,
                    HoodContants.kMinDegree,
                    HoodContants.kMaxDegree,
                    HoodContants.kPosThersholdDeg);

    private final Turret m_turret =
            new Turret(
                    TurretConstants.kTurretMotor,
                    TurretConstants.kFalconVelocityToDegpS,
                    TurretConstants.kFalconPositionToDegrees,
                    TurretConstants.kNominalVoltage,
                    GlobalConstants.kDt,
                    TurretConstants.kMaxDegree,
                    TurretConstants.kMinDegree,
                    TurretConstants.kLimitSwitch,
                    TurretConstants.kFeedForwards);

    private final Superstructure m_superstructure =
            new Superstructure(
                    m_pivotClimber, m_columnBottom, null, m_columnTop, m_intake, null, m_flywheel);
}
