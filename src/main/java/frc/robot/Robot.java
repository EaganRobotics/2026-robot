// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.infrastructure.RobotContainer;
import frc.lib.infrastructure.RobotInstance;
import frc.lib.replay.WPILogReadMACAddress;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.ironmaple.simulation.SimulatedArena;
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
  private Command testCommand;
  private RobotContainer robotContainer;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("RobotMACAddress", RobotInstance.getMacAddressStr());
    Logger.recordMetadata("RobotInstance", RobotInstance.getMacAddress().toString());
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Log active commands
    // from
    // https://github.com/Mechanical-Advantage/RobotCode2025Public/blob/aa2a88501601c3bac295cf80e46352c6b257088e/src/main/java/org/littletonrobotics/frc2025/Robot.java#L155-L171
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
      String name = command.getName();
      int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
      commandCounts.put(name, count);
      Logger.recordOutput("CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()),
          active);
      Logger.recordOutput("CommandsAll/" + name, count > 0);
    };
    CommandScheduler.getInstance()
        .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
    CommandScheduler.getInstance()
        .onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
    CommandScheduler.getInstance()
        .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

    // Record MAC address from log replay
    String macAddress = null;

    // Set up data receivers & replay source
    switch (SimConstants.CURRENT_MODE) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        // macAddress = WPILogReadMACAddress.get(logPath);
        macAddress = "00-80-2F-36-FD-D6";
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    switch (SimConstants.CURRENT_MODE) {
      case REAL:
      case SIM:
        // Instantiate our RobotContainer. This will perform all our button bindings,
        // and put our autonomous chooser on the dashboard.

        // Choose robot instance based on current MAC address
        robotContainer = RobotInstance.config(this::getRobotContainerFromInstance);
        break;

      case REPLAY:
        /*
         * Autodetects which robot instance to construct for REPLAY mode based on metadata
         */
        if (macAddress == null) {
          /*
           * IF REPLAYING A LOG MISSING MAC ADDRESS, MANUALLY SELECT CORRECT ROBOT CODE HERE
           */
          // robotContainer = new frc.robot.Robot24.RobotContainer();
          throw new RuntimeException("No MAC address in replay log");
        } else {
          robotContainer = getRobotContainerFromInstance(RobotInstance.fromString(macAddress));
        }
        break;
    }

    // Start AdvantageKit logger
    Logger.start();
  }

  private RobotContainer getRobotContainerFromInstance(RobotInstance instance) {
    return switch (instance) {
      // case Robot25 -> new frc.robot.Robot25.RobotContainer();
      case Robot26 -> new frc.robot.robot26.RobotContainer();
      case Simulator -> SimConstants.SIM_ROBOT_SUPPLIER.get();
      default -> throw new RuntimeException("Unsupported robot instance: " + instance.toString());
    };
  }

  /**
   * This function is called once when the robot is first started up. All robot-wide initialization
   * goes here.
   */
  @Override
  public void robotInit() {
    robotContainer.robotInit();
    WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
  }

  /** This function is called periodically during all modes. */
  @Override
  public void robotPeriodic() {
    // Switch thread to high priority to improve loop timing
    // TODO: AKit recommends you remove this unless you know your loop times are <0.02s
    Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to normal thread priority
    // TODO: This too (AKit)
    Threads.setCurrentThreadPriority(false, 10);

    robotContainer.robotPeriodic();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    robotContainer.disabledInit();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    robotContainer.disabledPeriodic();
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    testCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (testCommand != null) {
      testCommand.schedule();
    }

    robotContainer.autonomousInit();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    robotContainer.autonomousPeriodic();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (testCommand != null) {
      testCommand.cancel();
    }

    robotContainer.teleopInit();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    robotContainer.teleopPeriodic();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();
    testCommand = robotContainer.getTestCommand();

    // schedule the autonomous command (example)
    if (testCommand != null) {
      testCommand.schedule();
    }

    robotContainer.testInit();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    robotContainer.testPeriodic();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    robotContainer.resetSimulation();
    robotContainer.simulationInit();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    robotContainer.displaySimFieldToAdvantageScope();
    robotContainer.simulationPeriodic();
  }
}
