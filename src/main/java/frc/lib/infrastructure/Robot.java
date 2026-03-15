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

package frc.lib.infrastructure;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.simulation.SimConstants;
import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.AutoLogOutputManager;
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
    // Look for @AutoLogOutput everywhere
    AutoLogOutputManager.addPackage("frc");

    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("RobotMACAddress", RobotInstance.getMacAddressStr());
    // Logger.recordMetadata("RobotInstance",
    // RobotInstance.getMacAddress().toString());
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
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.recordOutput(
              "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
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

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer using ServiceLoader
    robotContainer = findRobotContainer(macAddress);
  }

  /** Use ServiceLoader to find the RobotContainer that matches the current robot. */
  private RobotContainer findRobotContainer(String macAddress) {
    if (SimConstants.CURRENT_MODE == SimConstants.Mode.SIM) {
      RobotContainer container = SimConstants.SIM_ROBOT_SUPPLIER.get();
      container.initialize();
      return container;
    }

    // In replay mode, macAddress might be null if not in log
    // We can fallback to searching providers or error out if critical
    String searchAddress = (macAddress != null) ? macAddress : RobotInstance.getMacAddressStr();

    java.util.ServiceLoader<RobotContainer> loader =
        java.util.ServiceLoader.load(RobotContainer.class);

    for (RobotContainer container : loader) {
      if (container.matchesMacAddress(searchAddress)) {
        System.out.println("Found RobotContainer: " + container.getRobotName());
        container.initialize();
        return container;
      }
    }

    throw new RuntimeException("No RobotContainer found for MAC address: " + searchAddress);
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
    // AKit recommends you remove this unless you know your loop times are
    // <0.02s
    Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Return to normal thread priority
    // This too (AKit)
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

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    testCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (testCommand != null) {
      CommandScheduler.getInstance().schedule(testCommand);
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
    // System.out.println(
    //     SnapCommands.isInCircularZone(
    //         robotContainer.getRobotPose(), new Translation2d(4.5, 5.6), Meters.of(0.6)));
    // System.out.println(robotContainer.getRobotPose());
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();
    testCommand = robotContainer.getTestCommand();

    // schedule the autonomous command (example)
    if (testCommand != null) {
      CommandScheduler.getInstance().schedule(testCommand);
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
    robotContainer.simulationInit();
    robotContainer.resetSimulation();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    SimulatedArena.getInstance().simulationPeriodic();
    robotContainer.displaySimFieldToAdvantageScope();
    robotContainer.simulationPeriodic();
  }
}
