package frc.robot26;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.simulation.SimConstants;
import frc.robot26.commands.DriveCommands;
import frc.robot26.commands.RollerCommands;
import frc.robot26.commands.ShooterCommands;
import frc.robot26.commands.SnapCommands;
import frc.robot26.generated.TunerConstants;
import frc.robot26.subsystems.drive.Drive;
import frc.robot26.subsystems.drive.DriveSimConfig;
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
import frc.robot26.subsystems.intake.IntakeIO;
import frc.robot26.subsystems.intake.IntakeIOSim;
import frc.robot26.subsystems.intake.IntakeIOTalonFX;
import frc.robot26.subsystems.shooter.Shooter;
import frc.robot26.subsystems.shooter.ShooterConstants;
import frc.robot26.subsystems.shooter.ShooterIO;
import frc.robot26.subsystems.shooter.ShooterIOSim;
import frc.robot26.subsystems.shooter.ShooterIOTalonFX;
import frc.robot26.subsystems.vision.Vision;
import frc.robot26.subsystems.vision.Vision.VisionConsumer;
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
  private final CommandXboxController jithinController = new CommandXboxController(2);

  // Drive simulation
  private static final SwerveDriveSimulation driveSimulation =
      new SwerveDriveSimulation(
          DriveSimConfig.MAPLE_SIM_CONFIG, SimConstants.SIM_INITIAL_FIELD_POSE);

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
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight),
                driveSimulation::setSimulationWorldPose);
        vision =
            new Vision(
                new VisionConsumer() {
                  public void accept(
                      Pose2d visionRobotPoseMeters,
                      double timestampSeconds,
                      Matrix<N3, N1> visionMeasurementStdDevs) {
                    drive.addVisionMeasurement(
                        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
                  }
                },
                new VisionIOLimelight(
                    VisionConstants.limelightBack, () -> drive.getPose().getRotation()),
                new VisionIOLimelight(
                    VisionConstants.limelightTop, () -> drive.getPose().getRotation())
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
                new VisionConsumer() {
                  public void accept(
                      Pose2d visionRobotPoseMeters,
                      double timestampSeconds,
                      Matrix<N3, N1> visionMeasurementStdDevs) {
                    drive.addVisionMeasurement(
                        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
                  }
                },
                new VisionIOPhotonVisionSim(
                    VisionConstants.limelightBack,
                    VisionConstants.robotToCameraBack,
                    driveSimulation::getSimulatedDriveTrainPose),
                new VisionIOPhotonVisionSim(
                    VisionConstants.limelightTop,
                    VisionConstants.robotToCameraTop,
                    driveSimulation::getSimulatedDriveTrainPose));
        intake = new Intake(new IntakeIOSim(RobotContainer.driveSimulation));
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

    // NamedCommands.registerCommand(
    //     "IntakeOut", intake.setDeployPosition(IntakeConstants.DeployState.EXTENDED));
    // NamedCommands.registerCommand(
    //     "IntakeIn", intake.setDeployPosition(IntakeConstants.DeployState.RETRACTED));
    NamedCommands.registerCommand(
        "IntakeOut", intake.setDeployOpenLoop(Volts.of(3)).withTimeout(2));
    NamedCommands.registerCommand(
        "IntakeIn", intake.setDeployOpenLoop(Volts.of(-3)).withTimeout(2));
    NamedCommands.registerCommand("FeederOut", feeder.setOpenLoop(Volts.of(3)));
    NamedCommands.registerCommand("FeederIn", feeder.setOpenLoop(Volts.of(-3)));
    NamedCommands.registerCommand("FloorOut", floor.setOpenLoop(Volts.of(3)));
    NamedCommands.registerCommand("FloorIn", floor.setOpenLoop(Volts.of(-3)));
    NamedCommands.registerCommand("ShooterOut", shooter.setShooterOpenLoop(Volts.of(3)));
    NamedCommands.registerCommand("ShooterIn", shooter.setShooterOpenLoop(Volts.of(-3)));
    NamedCommands.registerCommand("Intake", intake.setIntakeClosedLoop(RPM.of(0)).withTimeout(10));

    // NamedCommands.registerCommand(
    //     "AutoScore",
    //     ShooterCommands.shootAutoAim(shooter, intake, floor, feeder, drive).withTimeout(5));
    NamedCommands.registerCommand(
        "AutoScore",
        ShooterCommands.shootAutoAimContinuous(shooter, floor, feeder, drive).withTimeout(5));
    NamedCommands.registerCommand(
        "AngleToHub",
        SnapCommands.snapToAngle(
                drive,
                () -> {
                  Translation2d hubToRobot =
                      SnapCommands.getHubCenter().minus(drive.getPose().getTranslation());
                  double angleToRobot = Math.atan2(hubToRobot.getY(), hubToRobot.getX());
                  return new Rotation2d(angleToRobot);
                })
            .withTimeout(.75));

    // autos wont work till intake out works

    NamedCommands.registerCommand(
        "AutoShootT17",
        RollerCommands.shootClosedLoop(
                shooter, floor, feeder, RPM.of(500), RPM.of(1000), RPM.of(1000))
            .withTimeout(17));
    NamedCommands.registerCommand(
        "SuperAutoShootT17",
        Commands.sequence(
            SnapCommands.snapToRadius(drive, Meters.of(1.5)),
            RollerCommands.shootClosedLoop(
                    shooter, floor, feeder, RPM.of(550), RPM.of(1000), RPM.of(1000))
                .withTimeout(17)));
    NamedCommands.registerCommand(
        "AutoShootT5",
        RollerCommands.shootClosedLoop(
                shooter, floor, feeder, RPM.of(500), RPM.of(1000), RPM.of(1000))
            .withTimeout(5));

    // NamedCommands.registerCommand(
    // "AutoShoot", RollerCommands.shootOpenLoop(shooter, floor, feeder, intake).withTimeout(3));

    // NamedCommands.registerCommand("SnapToRadius", DriveCommands.snapToRadius(drive,
    // Feet.of(10.0)));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addOption(
        "Center-Hub",
        RollerCommands.shootClosedLoop(
                shooter, floor, feeder, RPM.of(515), RPM.of(1000), RPM.of(1500))
            .withTimeout(17));

    autoChooser.addOption(
        "Angle-Hub",
        RollerCommands.shootClosedLoop(
                shooter, floor, feeder, RPM.of(545), RPM.of(1000), RPM.of(1500))
            .withTimeout(17));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
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
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));
    // shooter.setDefaultCommand(
    //     ShooterCommands.shooterDefaultCommand(
    //         shooter,
    //         () -> SnapCommands.distanceToHub(drive),
    //         () -> -driverController.getRightY(),
    //         Feet.of(5)));
    intake.setDefaultCommand(intake.setJoystickOpenLoop(() -> -operatorController.getRightY()));
    feeder.setDefaultCommand(
        feeder.setJoystickOpenLoop(() -> -operatorController.getRightY() * .85));
    floor.setDefaultCommand(floor.setJoystickOpenLoop(() -> -operatorController.getRightY() * .85));
    // shooter.setDefaultCommand(
    //     shooter.setHoodJoystickOpenLoop(() -> -operatorController.getLeftY() * .5));

    // Driver Controls

    // Lock to 0° when A button is held
    driverController
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(),
                () -> {
                  Translation2d hubToRobot =
                      SnapCommands.getHubCenter().minus(drive.getPose().getTranslation());
                  double angleToRobot = Math.atan2(hubToRobot.getY(), hubToRobot.getX());
                  return new Rotation2d(angleToRobot);
                }));

    // Switch to X pattern when X button is pressed
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    driverController
        .start()
        .or(driverController.back())
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // driverController.x().whileTrue(shooter.setShooterClosedLoop(RPM.of(2000)));
    // driverController
    //     .leftTrigger()
    //     .whileTrue(ShooterCommands.shootAutoAim(shooter, floor, feeder, drive));
    // driverController
    //     .rightTrigger()
    //     .whileTrue(ShooterCommands.shootManualAim(shooter, floor, feeder, drive));

    // driverController.a().whileTrue(feeder.setTunableFeeder());
    // driverController.x().whileTrue(SnapCommands.tuneableFlipitySnipitySnap(drive));
    // driverController.b().whileTrue(shooter.setTunableShooter());
    // driverController.b().whileTrue(floor.setTunableFloor());
    // driverController.y().whileTrue(RollerCommands.shootOpenLoop(floor, feeder));

    driverController.b().whileTrue(SnapCommands.snapToRadius(drive, Meters.of(3.5)));
    driverController.y().whileTrue(SnapCommands.snapToRadius(drive, Meters.of(1.5)));

    // driverController.y().whileTrue(RollerCommands.intakeJiggleOpenLoop(intake));
    // driverController.a().whileTrue(RollerCommands.intakeJiggleOpenLoop(intake));

    // 3.5m
    // driverController
    //     .x()
    //     .whileTrue(
    //         Commands.sequence(
    //             RollerCommands.shootClosedLoop(
    //                 shooter,
    //                 floor,
    //                 feeder,
    //                 RPM.of(750),
    //                 RPM.of(1000),
    //                 RPM.of(1000)))); // shooter then feeder

    // Operator Controls

    // operatorController.b().whileTrue(intake.setDeployOpenLoop(Volts.of(4)));
    operatorController.b().whileTrue(intake.setDeployOpenLoop(Volts.of(4)));
    operatorController.x().whileTrue(intake.setDeployOpenLoop(Volts.of(-4)));
    operatorController.leftTrigger().whileTrue(intake.setIntakeClosedLoop(RPM.of(7000)));
    operatorController
        .y()
        .whileTrue(
            shooter
                .setHoodPosition(Radians.of(0))
                .andThen(
                    RollerCommands.shootClosedLoopDangerous(
                        shooter, floor, feeder, RPM.of(500), RPM.of(4000), RPM.of(4000))));
    operatorController.a().whileTrue(shooter.setTunableShooter());

    // operatorController.y().whileTrue(RollerCommands.intakeJiggleOpenLoop(intake));

    // operatorController
    //     .a()
    //     .whileTrue(
    //         shooter
    //             .setHoodPosition(Radians.of(9.2))
    //             .andThen(
    //                 RollerCommands.shootClosedLoopDangerous(
    //                     shooter, floor, feeder, RPM.of(550), RPM.of(1000), RPM.of(2000))));

    operatorController.povUp().onTrue(shooter.incrementSetHoodPosition(Degrees.of(100)));
    operatorController.povDown().onTrue(shooter.incrementSetHoodPosition(Degrees.of(-100)));

    // operatorController.povUp().onTrue(shooter.setTunableHood());
    // operatorController.povDown().onTrue(shooter.setTunableHoodBack());

    // operatorController.povUp().onTrue(intake.setTunableIntakeDeploy());
    // operatorController.povDown().onTrue(intake.setTunableIntakeDeployBack());

    operatorController
        .rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  ShooterConstants.Real.shooterSpeed.incrementBy(15);
                }));
    operatorController
        .leftBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  ShooterConstants.Real.shooterSpeed.incrementBy(-15);
                }));

    operatorController
        .rightTrigger()
        .whileTrue(
            RollerCommands.tuneableShootClosedLoop(
                shooter, floor, feeder, RPM.of(1000), RPM.of(2000)));

    // operatorController
    //     .leftTrigger()
    //     .onTrue(intake.setDeployPosition(IntakeConstants.DeployState.RETRACTED));
    // operatorController
    //     .rightTrigger()
    //     .onTrue(intake.setDeployPosition(IntakeConstants.DeployState.EXTENDED));

    // operatorController.leftBumper().whileTrue(floor.setOpenLoop(Volts.of(3)));
    // operatorController.rightBumper().whileTrue(floor.setOpenLoop(Volts.of(-3)));

    // operatorController.leftTrigger().whileTrue(intake.setTunableIntake());

    // operatorController.povUp().whileTrue(shooter.setShooterOpenLoop(Volts.of(3)));
    // operatorController.povDown().whileTrue(shooter.setShooterOpenLoop(Volts.of(-3)));

    // operatorController.povLeft().whileTrue(shooter.setTunableHood());

    // operatorController.povUp().whileTrue(shooter.setHoodOpenLoop(Volts.of(2)));
    // operatorController.povDown().whileTrue(shooter.setHoodOpenLoop(Volts.of(-2)));

    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive,
    //         () -> -jithinController.getLeftY(),
    //         () -> -jithinController.getLeftX(),
    //         () -> -jithinController.getRightX()));

    jithinController.leftTrigger().whileTrue(intake.setTunableIntake());
    jithinController.leftBumper().whileTrue(floor.setTunableFloor());
    jithinController.rightTrigger().whileTrue(shooter.setTunableShooter());
    jithinController.rightBumper().whileTrue(feeder.setTunableFeeder());

    jithinController.povUp().whileTrue(intake.setDeployClosedLoop(Inches.of(15)));
    jithinController.povLeft().whileTrue(intake.setDeployOpenLoop(Volts.of(-5)));
    jithinController.povRight().whileTrue(intake.setDeployOpenLoop(Volts.of(5)));

    jithinController
        .a()
        .whileTrue(
            ShooterCommands.shootAutoAim(shooter, floor, feeder, drive)
                .alongWith(RollerCommands.intakeJiggleOpenLoop(intake)));

    jithinController
        .b()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -jithinController.getLeftY(),
                () -> -jithinController.getLeftX(),
                () -> {
                  Translation2d hubToRobot =
                      SnapCommands.getHubCenter().minus(drive.getPose().getTranslation());
                  double angleToRobot = Math.atan2(hubToRobot.getY(), hubToRobot.getX());
                  return new Rotation2d(angleToRobot);
                }));
  }

  @Override
  public void teleopInit() {
    // drive.swerveBreak(true);
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
  public void robotPeriodic() {
    Logger.recordOutput(
        "Drive/DistanceToHubCenterFeet", SnapCommands.distanceToHub(drive).in(Feet));
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser != null ? autoChooser.get() : Commands.none();
  }

  @Override
  public Command getTestCommand() {
    return Commands.print("Test command!");
  }

  @Override
  public Pose2d getRobotPose() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getRobotPose'");
  }
}
