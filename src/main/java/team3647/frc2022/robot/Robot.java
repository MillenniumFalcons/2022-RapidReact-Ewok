// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package team3647.frc2022.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import team3647.lib.NetworkColorSensor.Color;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    public static final double kTenMSLoopTime = 0.01;
    public static final double kTwentyMSLoopTime = 0.02;

    public Alliance allianceColor = Alliance.Invalid;

    private RobotContainer m_robotContainer = new RobotContainer();
    int lastId = 0;

    public Robot() {
        super(.02);
        addPeriodic(
                m_robotContainer.m_drivetrain::readPeriodicInputs,
                kTenMSLoopTime,
                .004); // 2.5MS offset
        addPeriodic(
                m_robotContainer.m_turret::readPeriodicInputs,
                kTenMSLoopTime,
                .01); // 5.0 MS offset
        addPeriodic(
                m_robotContainer.m_flightDeck.getTracker()::update,
                kTenMSLoopTime,
                .016); // 7.5 MS offset
        addPeriodic(
                () -> m_robotContainer.m_superstructure.periodic(Timer.getFPGATimestamp()),
                kTwentyMSLoopTime,
                0.019);
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        LiveWindow.disableAllTelemetry();
        LiveWindow.setEnabled(false);
        // SmartDashboard.updateValues();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        // m_robotContainer.statusLED.setColor(StatusLED., blinkSpeed);
    }

    /** This function is called once each time the robot enters Disabled mode. */
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        this.m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        disableColorRejection();
        // schedule the autonomous command (example)
        System.out.println("Auto enabled!");
        this.m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            System.out.println("We have auto!");
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        updateColor();
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        m_robotContainer.m_superstructure.enableCompressor().schedule();
        // CommandScheduler.getInstance()
        //         .schedule(new InstantCommand(() -> {}, m_robotContainer.m_turret));
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    private void updateColor() {
        allianceColor = DriverStation.getAlliance();
        switch (allianceColor) {
            case Blue:
                m_robotContainer.m_superstructure.ourColor = Color.BLUE;
                break;
            case Red:
                m_robotContainer.m_superstructure.ourColor = Color.RED;
                break;
        }
    }

    private void disableColorRejection() {
        m_robotContainer.m_superstructure.ourColor = Color.NONE;
    }
}
