// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @SuppressWarnings("java:S1075")
  private static final String DEFAULT_PATH_RIO = "/home/lvuser/logs";

  private static final String DEFAULT_PATH_SIM = "logs";

  public Robot() {
    configureLogging();
    configureDriveStation();
    configureMotorLogging();
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  private void configureLogging() {
    if (Constants.RobotState.getMode().isRealtime()) {
      configureRealtimeLogging();
    } else {
      configureNonRealtimeLogging();
    }
    Logger.start();
  }

  private void configureRealtimeLogging() {
    setUseTiming(true); // Run at standard robot speed (20 ms)
    Logger.addDataReceiver(
        new WPILOGWriter(RobotBase.isSimulation() ? DEFAULT_PATH_SIM : DEFAULT_PATH_RIO));
    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
  }

  @SuppressWarnings("java:S1125")
  private void configureNonRealtimeLogging() {
    setUseTiming(
        Constants.RobotState.getMode() == Constants.RobotState.Mode.TEST
            ? true /* TODO: Still looking into ways to speed up Phoenix sim (false is faster) */
            : false); // Run as fast as possible

    if (Constants.RobotState.getMode() == Constants.RobotState.Mode.REPLAY) {
      configureReplayLogging();
    } else if (Constants.RobotState.getMode() == Constants.RobotState.Mode.TEST) {
      configureTestLogging();
    }
  }

  private static void configureReplayLogging() {
    String logPath =
        LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
    Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
    Logger.addDataReceiver(
        new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
  }

  private static void configureTestLogging() {
    if (Constants.RobotState.UNIT_TESTS_ENABLE_ADVANTAGE_SCOPE) {
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    }
  }

  private static void configureDriveStation() {
    DriverStation.silenceJoystickConnectionWarning(
        Constants.RobotState.getMode() != Constants.RobotState.Mode.REAL);
  }

  private static void configureMotorLogging() {
    if (Constants.RobotState.MOTOR_LOGGING_ENABLED) {
      for (int i = 0; i < 10; i++) { // NOSONAR
        DriverStation.reportWarning(
            "[WARNING] Motor logging enabled, DON'T FORGET to delete old logs to make space on disk.\n"
                + "[WARNING] During competition, set MOTOR_LOGGING_ENABLED to false since logging is enabled automatically.",
            false);
      }
      if (Constants.RobotState.getMode() != Constants.RobotState.Mode.TEST) {
        SignalLogger.start();
      }
    }
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
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.teleopInit();
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
  public void testPeriodic() {
    m_robotContainer.testPeriodic();
  }
}
