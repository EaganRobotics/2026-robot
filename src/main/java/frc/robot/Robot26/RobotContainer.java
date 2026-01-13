package frc.robot.robot26;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.SimConstants;
import frc.robot.robot26.commands.DriveCommands;
import frc.robot.robot26.subsystems.drive.Drive;
import frc.robot.robot26.subsystems.drive.GyroIOSim;
import frc.robot.robot26.subsystems.drive.ModuleIOSim;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer extends frc.lib.infrastructure.RobotContainer {

  // Subsystems
  private final Drive drive;

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);

  // Drive simulation
  private static final SwerveDriveSimulation driveSimulation =
      new SwerveDriveSimulation(Drive.MAPLE_SIM_CONFIG, SimConstants.SIM_INITIAL_FIELD_POSE);

  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    super(driveSimulation);

    // Create IO implementations
    switch (SimConstants.CURRENT_MODE) {
      case REAL:
        drive = null;
        break;
      case SIM:
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        drive.setPose(SimConstants.SIM_INITIAL_FIELD_POSE);
        break;
      case REPLAY:
        drive = null;
        break;
      default:
        drive = null;
        System.err.println("SimConstants.CURRENT_MODE was invalid");
        System.exit(1);
        break;
    }

    autoChooser =
        new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser("AL.0C.1M"));

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            driverController::getLeftY,
            driverController::getLeftX,
            () -> -driverController.getRightX() * .85));
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  @Override
  public Command getTestCommand() {
    return Commands.print("Test command!");
  }
}
