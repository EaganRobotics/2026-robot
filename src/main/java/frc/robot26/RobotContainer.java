package frc.robot26;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.simulation.SimConstants;
import frc.robot26.commands.DriveCharacterization;
import frc.robot26.commands.DriveCommands;
import frc.robot26.commands.RollerCommands;
import frc.robot26.commands.ShooterCommands;
import frc.robot26.subsystems.drive.Drive;
import frc.robot26.subsystems.drive.DriveConstants;
import frc.robot26.subsystems.drive.GyroIO;
import frc.robot26.subsystems.drive.GyroIOPigeon2;
import frc.robot26.subsystems.drive.GyroIOSim;
import frc.robot26.subsystems.drive.ModuleIO;
import frc.robot26.subsystems.drive.ModuleIOSim;
import frc.robot26.subsystems.drive.ModuleIOTalonFX;
import frc.robot26.subsystems.feeder.Feeder;
import frc.robot26.subsystems.feeder.FeederIO;
import frc.robot26.subsystems.feeder.FeederIOSim;
import frc.robot26.subsystems.feeder.FeederIOTalonFX;
import frc.robot26.subsystems.floor.Floor;
import frc.robot26.subsystems.floor.FloorIO;
import frc.robot26.subsystems.floor.FloorIOSim;
import frc.robot26.subsystems.floor.FloorIOTalonFX;
import frc.robot26.subsystems.intake.Intake;
import frc.robot26.subsystems.intake.IntakeConstants;
import frc.robot26.subsystems.intake.IntakeIO;
import frc.robot26.subsystems.intake.IntakeIOSim;
import frc.robot26.subsystems.intake.IntakeIOTalonFX;
import frc.robot26.subsystems.shooter.Shooter;
import frc.robot26.subsystems.shooter.ShooterIO;
import frc.robot26.subsystems.shooter.ShooterIOSim;
import frc.robot26.subsystems.shooter.ShooterIOTalonFX;
import frc.robot26.subsystems.vision.Vision;
import frc.robot26.subsystems.vision.VisionConstants;
import frc.robot26.subsystems.vision.VisionIOLimelight;
import frc.robot26.subsystems.vision.VisionIOPhotonVisionSim;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

@SuppressFBWarnings("NM_SAME_SIMPLE_NAME_AS_SUPERCLASS")
public class RobotContainer extends frc.lib.infrastructure.RobotContainer {

  // Subsystems
  private Drive drive;
  private Intake intake;
  private Feeder feeder;
  private Floor floor;
  private Shooter shooter;

  @SuppressFBWarnings("URF_UNREAD_FIELD")
  private Vision vision;

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

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
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(DriveConstants.FrontLeft),
                new ModuleIOTalonFX(DriveConstants.FrontRight),
                new ModuleIOTalonFX(DriveConstants.BackLeft),
                new ModuleIOTalonFX(DriveConstants.BackRight),
                driveSimulation::setSimulationWorldPose);
        vision =
            new Vision(
                drive,
                new VisionIOLimelight(
                    VisionConstants.limelightShooter, () -> drive.getPose().getRotation())
                // , new VisionIOLimelight(VisionConstants.limelightBack,
                // () -> drive.getPose().getRotation())
                );
        intake = new Intake(new IntakeIOTalonFX());
        feeder = new Feeder(new FeederIOTalonFX());
        floor = new Floor(new FloorIOTalonFX());
        shooter = new Shooter(new ShooterIOTalonFX());
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
        feeder = new Feeder(new FeederIOSim());
        floor = new Floor(new FloorIOSim());
        shooter = new Shooter(new ShooterIOSim());
        break;
      case REPLAY:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                driveSimulation::setSimulationWorldPose);
        vision = null;
        intake = new Intake(new IntakeIO() {});
        feeder = new Feeder(new FeederIO() {});
        floor = new Floor(new FloorIO() {});
        shooter = new Shooter(new ShooterIO() {});
        break;
      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                driveSimulation::setSimulationWorldPose);
        vision = null;
        intake = new Intake(new IntakeIO() {});
        feeder = new Feeder(new FeederIO() {});
        floor = new Floor(new FloorIO() {});
        shooter = new Shooter(new ShooterIO() {});
        throw new IllegalStateException(
            "SimConstants.CURRENT_MODE was invalid: " + SimConstants.CURRENT_MODE);
    }

    NamedCommands.registerCommand(
        "IntakeOut", intake.setDeployPosition(IntakeConstants.DeployState.EXTENDED));
    NamedCommands.registerCommand(
        "IntakeIn", intake.setDeployPosition(IntakeConstants.DeployState.RETRACTED));
    NamedCommands.registerCommand("FeederOut", feeder.setOpenLoop(Volts.of(3)));
    NamedCommands.registerCommand("FeederIn", feeder.setOpenLoop(Volts.of(-3)));
    NamedCommands.registerCommand("FloorOut", floor.setOpenLoop(Volts.of(3)));
    NamedCommands.registerCommand("FloorIn", floor.setOpenLoop(Volts.of(-3)));
    NamedCommands.registerCommand("ShooterOut", shooter.setShooterOpenLoop(Volts.of(3)));
    NamedCommands.registerCommand("ShooterIn", shooter.setShooterOpenLoop(Volts.of(-3)));

    NamedCommands.registerCommand(
        "AutoShoot",
        RollerCommands.shootClosedLoop(
            shooter, floor, feeder, RPM.of(500), RPM.of(1000), RPM.of(1000)));

    // NamedCommands.registerCommand(
    // "AutoShoot", RollerCommands.shootOpenLoop(shooter, floor, feeder, intake).withTimeout(3));

    NamedCommands.registerCommand("SnapToRadius", DriveCommands.snapToRadius(drive, Feet.of(10.0)));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addOption(
        "Week0-Test",
        RollerCommands.shootClosedLoop(
                shooter, floor, feeder, RPM.of(515), RPM.of(1000), RPM.of(1500))
            .withTimeout(17));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        DriveCharacterization.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization",
        DriveCharacterization.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
      shooter.setDefaultCommand(
          shooter.setShooterJoystickOpenLoop(() -> -operatorController.getLeftY() * .5));
      intake.setDefaultCommand(intake.setJoystickOpenLoop(() -> -operatorController.getRightY()));
      feeder.setDefaultCommand(
          feeder.setJoystickOpenLoop(() -> -operatorController.getRightY() * .85));
      floor.setDefaultCommand(
          floor.setJoystickOpenLoop(() -> -operatorController.getRightY() * .85));
    }

    // Driver Controls

    driverController
        .start()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      drive.setPose(new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero));
                    },
                    drive)
                .ignoringDisable(true)
                .withName("RobotContainer.driverZeroCommand"));

    driverController.x().whileTrue(shooter.setShooterClosedLoop(RPM.of(2000)));
    driverController
        .leftTrigger()
        .whileTrue(ShooterCommands.shootAutoAim(shooter, floor, feeder, drive));
    driverController
        .rightTrigger()
        .whileTrue(ShooterCommands.shootManualAim(shooter, floor, feeder, drive));

    driverController.a().whileTrue(feeder.setTunableFeeder());
    // driverController.x().whileTrue(DriveCommands.snapToRadiusInterpolation(drive, Feet.of(7.0)));
    // driverController.b().whileTrue(shooter.setTunableShooter());
    driverController.b().whileTrue(floor.setTunableFloor());
    driverController.y().whileTrue(RollerCommands.shootOpenLoop(floor, feeder));
    driverController
        .x()
        .whileTrue(
            Commands.sequence(
                RollerCommands.shootClosedLoop(
                    shooter,
                    floor,
                    feeder,
                    RPM.of(750),
                    RPM.of(1000),
                    RPM.of(1000)))); // shooter then feeder

    // Operator Controls

    operatorController.b().whileTrue(intake.setDeployOpenLoop(Volts.of(2)));
    operatorController.x().whileTrue(intake.setDeployOpenLoop(Volts.of(-2)));

    operatorController.a().whileTrue(intake.setTunableIntake());
    operatorController
        .y()
        .whileTrue(
            Commands.sequence(
                RollerCommands.shootClosedLoop(
                    shooter, floor, feeder, RPM.of(550), RPM.of(1000), RPM.of(2000))));

    operatorController
        .leftTrigger()
        .onTrue(intake.setDeployPosition(IntakeConstants.DeployState.RETRACTED));
    operatorController
        .rightTrigger()
        .onTrue(intake.setDeployPosition(IntakeConstants.DeployState.EXTENDED));

    operatorController.leftBumper().whileTrue(floor.setOpenLoop(Volts.of(3)));
    operatorController.rightBumper().whileTrue(floor.setOpenLoop(Volts.of(-3)));

    operatorController.povUp().whileTrue(shooter.setShooterOpenLoop(Volts.of(3)));
    operatorController.povDown().whileTrue(shooter.setShooterOpenLoop(Volts.of(-3)));
  }

  @Override
  public void teleopInit() {
    drive.swerveBreak(true);
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
