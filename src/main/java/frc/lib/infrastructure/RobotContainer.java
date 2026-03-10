package frc.lib.infrastructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.simulation.SimConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.Logger;

public abstract class RobotContainer {

  private AbstractDriveTrainSimulation driveSimulation = null;

  protected RobotContainer() {}

  public abstract String getRobotName();

  public abstract boolean matchesMacAddress(String macAddress);

  public abstract void initialize();

  public abstract Pose2d getRobotPose();

  public void configureDriveSimulation(AbstractDriveTrainSimulation driveSimulation) {
    this.driveSimulation = driveSimulation;
    SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

    System.out.println("Drive simulation configured");
  }

  public abstract Command getTestCommand();

  public abstract Command getAutonomousCommand();

  public void robotInit() {}

  public void robotPeriodic() {}

  public void disabledInit() {}

  public void disabledPeriodic() {}

  public void autonomousInit() {}

  public void autonomousPeriodic() {}

  public void teleopInit() {}

  public void teleopPeriodic() {}

  public void testInit() {}

  public void testPeriodic() {}

  public void simulationInit() {}

  public void simulationPeriodic() {}

  public void resetSimulation() {
    if (SimConstants.CURRENT_MODE != SimConstants.Mode.SIM) return;

    if (driveSimulation != null)
      driveSimulation.setSimulationWorldPose(SimConstants.SIM_INITIAL_FIELD_POSE);
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public final void displaySimFieldToAdvantageScope() {
    if (SimConstants.CURRENT_MODE != SimConstants.Mode.SIM) return;

    if (driveSimulation != null)
      Logger.recordOutput(
          "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
  }
}
