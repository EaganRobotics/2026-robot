package frc.robot26;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.infrastructure.Robot;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.TestInstance;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

// Boots the robot in simulation and runs 5 seconds of real-time Sim before passing if it has not
// caught fatal errors

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
class RobotSimulationTest {

  private Robot robot;

  @BeforeAll
  void setUp() {
    assert HAL.initialize(500, 0) : "HAL failed to initialize";

    DriverStationSim.setEnabled(true);
    DriverStationSim.setTest(false);
    DriverStationSim.setEStop(false);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.notifyNewData();

    // i think this constructs the robot
    robot = new Robot();
    robot.robotInit();
    robot.simulationInit();
  }

  @ParameterizedTest
  @ValueSource(booleans = {true, false})
  void robotSurvivesFiveSeconds(boolean isAuto) {
    assertDoesNotThrow(
        () -> {
          final int iterations =
              250; // 250 x 20ms = 5 seconds; this feels like a bad way to do it but this is what
          // you
          // get

          DriverStationSim.setAutonomous(isAuto);
          DriverStationSim.notifyNewData();

          if (isAuto) {
            robot.autonomousInit();
          } else {
            robot.teleopInit();
          }

          for (int i = 0; i < iterations; i++) {
            DriverStationSim.notifyNewData();
            robot.robotPeriodic();

            if (isAuto) {
              robot.autonomousPeriodic();
            } else {
              robot.teleopPeriodic();
            }

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
