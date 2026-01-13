package frc.robot.robot26;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.SimConstants;
import frc.robot.robot26.subsystems.drive.Drive;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer extends frc.lib.infrastructure.RobotContainer {

  // Drive simulation
  private static final SwerveDriveSimulation driveSimulation =
      new SwerveDriveSimulation(Drive.MAPLE_SIM_CONFIG, SimConstants.SIM_INITIAL_FIELD_POSE);

  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    super(driveSimulation);

    // Create IO implementations
    switch (SimConstants.CURRENT_MODE) {
      case REAL:
        break;
      case SIM:
        break;
      case REPLAY:
        break;
    }

    autoChooser =
        new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser("AL.0C.1M"));

    configureButtonBindings();
  }

  private void configureButtonBindings() {}

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  @Override
  public Command getTestCommand() {
    return Commands.print("Test command!");
  }
}
