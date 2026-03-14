package frc.robot26;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.infrastructure.Robot;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInstance;

// Boots the robot in simulation and runs 5 seconds of real-time Sim before passing if it has not
// caught fatal erros

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class RobotSimulationTest {

  private Robot robot;

  @BeforeAll
  void setUp() {
    assert HAL.initialize(500, 0) : "HAL failed to initialize";

    // we could make auto test but not a right now thing
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.setEStop(false);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();

    // i think this constructs the robot
    robot = new Robot();
    robot.robotInit();
    robot.simulationInit();
  }

  @Test
  void robotSurvivesFiveSeconds() {
    assertDoesNotThrow(
        () -> {
          final int iterations =
              150; // 150 x 20ms = 3 seconds this feels like a bad way to do it but this is what you
          // get

          for (int i = 0; i < iterations; i++) {
            robot.robotPeriodic();
            robot.teleopPeriodic();
            robot.simulationPeriodic();

            // Not a clue prob something with stability
            Thread.sleep(20);
          }
        },
        "Robot threw an exception during simulation test run");
  }

  @AfterAll
  void tearDown() {
    if (robot != null) {
      robot.endCompetition();
      robot.close();
    }
    CommandScheduler.getInstance().cancelAll();
  }
}
