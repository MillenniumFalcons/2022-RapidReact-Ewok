// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team3647.frc2022.commands.ArcadeDrive;
import team3647.frc2022.commands.OpenLoop;
import team3647.frc2022.constants.*;
import team3647.frc2022.subsystems.ColumnBottom;
import team3647.frc2022.subsystems.ColumnTop;
import team3647.frc2022.subsystems.Drivetrain;
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
    // The robot's subsystems and commands are defined here...
    // edu.wpi.first.wpilibj.PowerDistribution replaces PDP.java from 3647 lib
    // edu.wpi.first.wpilibj.Compressor replaces Compressor.java from 3647 lib
    // edu.wpi.first.wpilibj.Solenoid replaces Solenoid.java from 3647 lib
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

    // private final Intake m_intake =
    //         new Intake(
    //                 IntakeConstants.kIntakeMotor,
    //                 IntakeConstants.nativeVelToSurfaceMpS,
    //                 0,
    //                 IntakeConstants.kNominalVoltage,
    //                 GlobalConstants.kDt,
    //                 IntakeConstants.kFeedForward,
    //                 IntakeConstants.kPistons);

    private final ColumnTop m_columnTop =
            new ColumnTop(
                    ColumnTopConstants.kColumnMotor,
                    ColumnTopConstants.kNativeVelToSurfaceMpS,
                    ColumnTopConstants.kPosConverstion,
                    ColumnTopConstants.kNominalVoltage,
                    GlobalConstants.kDt,
                    ColumnTopConstants.kFeedForward);

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
                    VerticalRollersConstants.kColumnMotor,
                    VerticalRollersConstants.kNativeVelToSurfaceMpS,
                    VerticalRollersConstants.kPosConverstion,
                    VerticalRollersConstants.kNominalVoltage,
                    GlobalConstants.kDt,
                    VerticalRollersConstants.kFeedForward);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        m_drivetrain.init();
        m_commandScheduler.registerSubsystem(
                m_drivetrain, /*m_intake,*/
                m_printer,
                m_columnTop,
                m_columnBottom,
                m_verticalRollers);
        // Configure the button bindings
        m_drivetrain.setDefaultCommand(
                new ArcadeDrive(
                        m_drivetrain,
                        mainController::getLeftStickY,
                        mainController::getRightStickX));

        m_columnBottom.setDefaultCommand(new OpenLoop(m_columnBottom, coController::getLeftStickY));
        m_verticalRollers.setDefaultCommand(
                new OpenLoop(m_columnBottom, coController::getLeftStickY));
        m_columnTop.setDefaultCommand(new OpenLoop(m_columnBottom, coController::getLeftStickY));
        configureButtonBindings();
    }

    private void configureButtonBindings() {}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }
}
