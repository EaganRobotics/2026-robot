package frc.lib.infrastructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.SimConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

public abstract class RobotContainer {

  private SwerveDriveSimulation driveSimulation = null;

  protected RobotContainer(SwerveDriveSimulation driveSimulation) {
    this.driveSimulation = driveSimulation;

    if (SimConstants.CURRENT_MODE == SimConstants.Mode.SIM) {
      SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
    }
  }

  public abstract Command getTestCommand();

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public abstract Command getAutonomousCommand();

  /**
   * This function is called once when the robot is first started up. All robot-wide initialization
   * goes here.
   */
  public void robotInit() {}

  /** This function is called periodically during all modes. */
  public void robotPeriodic() {}

  /** This function is called once when the robot is disabled. */
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  public void disabledPeriodic() {}

  /** This function is called once when autonomous is enabled. */
  public void autonomousInit() {}

  /** This function is called periodically during autonomous. */
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  public void testInit() {}

  /** This function is called periodically during test mode. */
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  public void simulationPeriodic() {}

  public void resetSimulation() {
    if (SimConstants.CURRENT_MODE != SimConstants.Mode.SIM) return;

    driveSimulation.setSimulationWorldPose(SimConstants.SIM_INITIAL_FIELD_POSE);
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void displaySimFieldToAdvantageScope() {
    if (SimConstants.CURRENT_MODE != SimConstants.Mode.SIM) return;

    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
