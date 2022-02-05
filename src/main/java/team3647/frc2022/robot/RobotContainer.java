// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import team3647.frc2022.commands.ArcadeDrive;
import team3647.frc2022.commands.ClimberUpDown;
import team3647.frc2022.commands.IntakeBallTest;
import team3647.frc2022.commands.ShootBall;
import team3647.frc2022.constants.*;
import team3647.frc2022.subsystems.ClimberArm;
import team3647.frc2022.subsystems.ColumnBottom;
import team3647.frc2022.subsystems.ColumnTop;
import team3647.frc2022.subsystems.Drivetrain;
import team3647.frc2022.subsystems.Flywheel;
import team3647.frc2022.subsystems.Intake;
import team3647.frc2022.subsystems.PivotClimber;
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
        m_drivetrain.init();
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
                m_pivotClimber);
        // Configure the button bindings
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
        coController.rightTrigger.whenHeld(
                new ShootBall(m_flywheel, m_columnTop, m_columnBottom, 8.23));
        coController.leftTrigger.whenHeld(
                new IntakeBallTest(
                        m_intake,
                        m_verticalRollers,
                        m_columnBottom,
                        coController::getLeftTriggerValue));
        // coController.leftTrigger.whenReleased(new InstantCommand(m_intake::retract, m_intake));
        m_pivotClimber.setDefaultCommand(
                new ClimberUpDown(
                        m_pivotClimber,
                        mainController::getLeftTriggerValue,
                        mainController::getRightTriggerValue));
        configureButtonBindings();
        m_printer.addDouble("Shooter velocity", m_flywheel::getVelocity);
        m_printer.addDouble("Kicker Velocity", m_columnTop::getVelocity);
        m_printer.addDouble("Shooter current", m_flywheel::getMasterCurrent);
    }

    private void configureButtonBindings() {
        mainController.buttonX.whenActive(new InstantCommand(m_pivotClimber::setAngled));
        mainController.buttonB.whenActive(new InstantCommand(m_pivotClimber::setStraight));
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

    private final CommandScheduler m_commandScheduler = CommandScheduler.getInstance();

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
    private final VerticalRollers m_verticalRollers =
            new VerticalRollers(
                    VerticalRollersConstants.kColumnMotor,
                    VerticalRollersConstants.kNativeVelToSurfaceMpS,
                    VerticalRollersConstants.kPosConverstion,
                    VerticalRollersConstants.kNominalVoltage,
                    GlobalConstants.kDt,
                    VerticalRollersConstants.kFeedForward);

    private final ColumnBottom m_columnBottom =
            new ColumnBottom(
                    ColumnBottomConstants.kColumnMotor,
                    ColumnBottomConstants.kNativeVelToSurfaceMpS,
                    ColumnBottomConstants.kPosConverstion,
                    ColumnBottomConstants.kNominalVoltage,
                    GlobalConstants.kDt,
                    ColumnBottomConstants.kFeedForward);

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
}
