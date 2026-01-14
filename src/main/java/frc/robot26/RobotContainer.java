package frc.robot26;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.simulation.SimConstants;
import frc.robot26.commands.DriveCommands;
import frc.robot26.subsystems.IntakeAndShooterTestSubsystem;
import frc.robot26.subsystems.drive.Drive;
import frc.robot26.subsystems.drive.GyroIOSim;
import frc.robot26.subsystems.drive.ModuleIOSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer extends frc.lib.infrastructure.RobotContainer {

  // Subsystems
  private Drive drive;
  private IntakeAndShooterTestSubsystem testSubsystem;

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);

  // Drive simulation
  private static final SwerveDriveSimulation driveSimulation =
      new SwerveDriveSimulation(Drive.MAPLE_SIM_CONFIG, SimConstants.SIM_INITIAL_FIELD_POSE);

  private LoggedDashboardChooser<Command> autoChooser;

  @Override
  public String getRobotName() {
    return "Robot26";
  }

  @Override
  public boolean matchesMacAddress(String macAddress) {
    // Replace with actual MAC address for Robot26 when known
    return true;
  }

  @Override
  public void initialize() {
    // Create IO implementations
    switch (SimConstants.CURRENT_MODE) {
      case REAL:
        drive = null;
        break;
      case SIM:
        // driveSimulation is static, so we can use it here consistently
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);

        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        drive.setPose(SimConstants.SIM_INITIAL_FIELD_POSE);
        testSubsystem = new IntakeAndShooterTestSubsystem(driveSimulation);
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

  public RobotContainer() {
    // Constructor must be empty for ServiceLoader
  }

  private void configureButtonBindings() {
    if (drive != null) {
      drive.setDefaultCommand(
          DriveCommands.joystickDrive(
              drive,
              driverController::getLeftY,
              driverController::getLeftX,
              () -> -driverController.getRightX() * .85));
    }

    if (testSubsystem != null) {
      // X Button -> Toggle Intake ON/OFF
      driverController.x().onTrue(testSubsystem.toggleIntakeCommand());
      // Y Button -> Toggle Shooter ON/OFF
      driverController.y().onTrue(testSubsystem.toggleShooterCommand());
      // B Button -> Shoot (fire projectile)
      driverController.b().onTrue(testSubsystem.shootCommand());
      // A Button -> Reverse Intake (hold)
      driverController.a().whileTrue(testSubsystem.runIntakeCommand(-8.0));
      // Right Trigger -> Run Intake (hold)
      driverController.rightTrigger().whileTrue(testSubsystem.runIntakeCommand(8.0));
      // Left Trigger -> Run Shooter (hold)
      driverController.leftTrigger().whileTrue(testSubsystem.runShooterCommand(12.0));
    }
  }

  @Override
  public void simulationInit() {
    if (!(SimulatedArena.getInstance() instanceof Arena2026Rebuilt arena)) return;

    arena.getBlueHub().setOnScoredCallback((gp) -> System.out.println("Blue Hub Scored!"));
    arena.getRedHub().setOnScoredCallback((gp) -> System.out.println("Red Hub Scored!"));
    arena.setNeutralFuelCount(200);
  }

  @Override
  public void simulationPeriodic() {
    if (!(SimulatedArena.getInstance() instanceof Arena2026Rebuilt arena)) return;

    Logger.recordOutput(
        "FieldSimulation/FuelPositions", arena.getGamePieceManager().getPosesArrayByType("Fuel"));
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser != null ? autoChooser.get() : Commands.none();
  }

  @Override
  public Command getTestCommand() {
    return Commands.print("Test command!");
  }
}
