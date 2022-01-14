// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team3647.frc2022.commands.ArcadeDrive;
import team3647.frc2022.subsystems.Drivetrain;
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
                    Constants.CDrivetrain.kLeftMaster,
                    Constants.CDrivetrain.kRightMaster,
                    Constants.CDrivetrain.kLeftSlave,
                    Constants.CDrivetrain.kRightSlave,
                    Constants.CDrivetrain.kPigeonIMU,
                    Constants.CDrivetrain.kFeedforward,
                    Constants.CDrivetrain.kPoseEstimator,
                    Constants.CDrivetrain.kFalconVelocityToMpS,
                    Constants.CDrivetrain.kFalconTicksToMeters,
                    Constants.CDrivetrain.kNominalVoltage);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
        m_commandScheduler.setDefaultCommand(
                m_drivetrain,
                new ArcadeDrive(
                        m_drivetrain,
                        mainController::getLeftStickY,
                        mainController::getRightStickX));
        m_printer.addBoolean("rightMaster invert", m_drivetrain::getRightMasterInvert);
        m_printer.addBoolean("rightSlave invert", m_drivetrain::getRightSlaveInvert);
        m_printer.addDouble("Robot X", m_drivetrain::getDrivetrainXMeters);
        m_printer.addDouble("Robot Y", m_drivetrain::getDrivetrainYMeters);

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
