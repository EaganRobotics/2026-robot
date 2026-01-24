package frc.robot26;

import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.simulation.SimConstants;
import frc.robot26.commands.DriveCommands;
import frc.robot26.subsystems.drive.Drive;
import frc.robot26.subsystems.drive.GyroIOSim;
import frc.robot26.subsystems.drive.ModuleIOSim;
import frc.robot26.subsystems.intake.Intake;
import frc.robot26.subsystems.intake.IntakeIOSim;
import frc.robot26.subsystems.intake.IntakeIOTalonFX;
import frc.robot26.subsystems.vision.Vision;
import frc.robot26.subsystems.vision.VisionConstants;
import frc.robot26.subsystems.vision.VisionIOPhotonVisionSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer extends frc.lib.infrastructure.RobotContainer {

  // Subsystems
  private Drive drive;
  private Vision vision;
  private Intake intake;

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
        vision = null;
        intake = new Intake(new IntakeIOTalonFX());
        break;
      case SIM:
        super.configureDriveSimulation(driveSimulation);
        drive =
            new Drive(
                new GyroIOSim(driveSimulation.getGyroSimulation()),
                new ModuleIOSim(driveSimulation.getModules()[0]),
                new ModuleIOSim(driveSimulation.getModules()[1]),
                new ModuleIOSim(driveSimulation.getModules()[2]),
                new ModuleIOSim(driveSimulation.getModules()[3]),
                driveSimulation::setSimulationWorldPose);
        drive.setPose(SimConstants.SIM_INITIAL_FIELD_POSE);
        vision =
            new Vision(
                drive,
                new VisionIOPhotonVisionSim(
                    VisionConstants.limelightShooter,
                    VisionConstants.robotToCameraShooter,
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.limelightBack,
                    VisionConstants.robotToCameraBack,
                    driveSimulation::getSimulatedDriveTrainPose));
        intake = new Intake(new IntakeIOSim());
        break;
      case REPLAY:
        drive = null;
        vision = null;
        break;
      default:
        drive = null;
        vision = null;
        System.err.println("SimConstants.CURRENT_MODE was invalid");
        System.exit(1);
        break;
    }

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

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

    driverController.a().whileTrue(intake.setOpenLoop(Volts.of(3)));
  }

  @Override
  public void simulationInit() {
    if (!(SimulatedArena.getInstance() instanceof Arena2026Rebuilt arena)) return;

    arena.getBlueHub().setOnScoredCallback((gp) -> System.out.println("Blue Hub Scored!"));
    arena.getRedHub().setOnScoredCallback((gp) -> System.out.println("Red Hub Scored!"));
    arena.setNeutralFuelCount(80); // for performance
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
