package frc.robot26;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.RPM;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.limelight.LimelightHelpers;
import frc.lib.simulation.SimConstants;
import frc.robot26.commands.ControllerCommands;
import frc.robot26.commands.DriveCommands;
import frc.robot26.commands.EverythingCommands;
import frc.robot26.commands.LEDCommands;
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
import frc.robot26.subsystems.leds.LEDs;
import frc.robot26.subsystems.leds.LEDsIO;
import frc.robot26.subsystems.leds.LEDsIOCANdle;
import frc.robot26.subsystems.leds.LEDsIOSim;
import frc.robot26.subsystems.shooter.Shooter;
import frc.robot26.subsystems.shooter.ShooterIO;
import frc.robot26.subsystems.shooter.ShooterIOSim;
import frc.robot26.subsystems.shooter.ShooterIOTalonFX;
import frc.robot26.subsystems.vision.Vision;
import frc.robot26.subsystems.vision.Vision.VisionConsumer;
import frc.robot26.subsystems.vision.VisionConstants;
import frc.robot26.subsystems.vision.VisionIO;
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
  private LEDs leds;

  @SuppressWarnings("URF_UNREAD_FIELD")
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
                    VisionConstants.limelightFront, () -> drive.getPose().getRotation())
                // new VisionIOLimelight(
                //     VisionConstants.limelightSide, () -> drive.getPose().getRotation())
                // , new VisionIOLimelight(VisionConstants.limelightBack,
                // () -> drive.getPose().getRotation())
                );
        intake = new Intake(new IntakeIOTalonFX());
        feeder = new Feeder(new FeederIOTalonFX());
        floor = new Floor(new FloorIOTalonFX());
        shooter = new Shooter(new ShooterIOTalonFX());
        leds = new LEDs(new LEDsIOCANdle());
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

        boolean isCI = System.getenv("CI") != null;
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
                isCI
                    ? new VisionIO() {}
                    : new VisionIOPhotonVisionSim(
                        VisionConstants.limelightFront,
                        VisionConstants.robotToCameraTop,
                        driveSimulation::getSimulatedDriveTrainPose));
        intake = new Intake(new IntakeIOSim(RobotContainer.driveSimulation));
        feeder = new Feeder(new FeederIOSim());
        floor = new Floor(new FloorIOSim());
        shooter = new Shooter(new ShooterIOSim());
        leds = new LEDs(new LEDsIOSim());
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
                new VisionIO() {});
        intake = new Intake(new IntakeIO() {});
        feeder = new Feeder(new FeederIO() {});
        floor = new Floor(new FloorIO() {});
        shooter = new Shooter(new ShooterIO() {});
        leds = new LEDs(new LEDsIO() {});
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
                new VisionIO() {});
        intake = new Intake(new IntakeIO() {});
        feeder = new Feeder(new FeederIO() {});
        floor = new Floor(new FloorIO() {});
        shooter = new Shooter(new ShooterIO() {});
        leds = new LEDs(new LEDsIO() {});
        throw new IllegalStateException(
            "SimConstants.CURRENT_MODE was invalid: " + SimConstants.CURRENT_MODE);
    }

    // NamedCommands.registerCommand(
    // "IntakeOut", intake.setDeployPosition(IntakeConstants.DeployState.EXTENDED));
    // NamedCommands.registerCommand(
    // "IntakeIn", intake.setDeployPosition(IntakeConstants.DeployState.RETRACTED));
    NamedCommands.registerCommand(
        "IntakeOut", intake.setDeployOpenLoop(Volts.of(4)).withTimeout(2));
    NamedCommands.registerCommand(
        "IntakeIn", intake.setDeployOpenLoop(Volts.of(-4)).withTimeout(2));
    NamedCommands.registerCommand("FeederOut", feeder.setOpenLoop(Volts.of(3)));
    NamedCommands.registerCommand("FeederIn", feeder.setOpenLoop(Volts.of(-3)));
    NamedCommands.registerCommand("FloorOut", floor.setOpenLoop(Volts.of(3)));
    NamedCommands.registerCommand("FloorIn", floor.setOpenLoop(Volts.of(-3)));
    NamedCommands.registerCommand("ShooterOut", shooter.setShooterOpenLoop(Volts.of(3)));
    NamedCommands.registerCommand("ShooterIn", shooter.setShooterOpenLoop(Volts.of(-3)));
    NamedCommands.registerCommand(
        "Intake", intake.setIntakeClosedLoop(RPM.of(7000)).withTimeout(10));
    NamedCommands.registerCommand(
        "IntakeFor5Secs", intake.setIntakeClosedLoop(RPM.of(8000)).withTimeout(5));
    NamedCommands.registerCommand(
        "SnapToTrench",
        SnapCommands.snapToPosition(
            drive, new Pose2d(new Translation2d(3.300, 7.400), Rotation2d.fromDegrees(0))));

    // NamedCommands.registerCommand(
    //     "KansasLeftFastSnap",
    //     SnapCommands.snapToPosition(
    //             drive, new Pose2d(new Translation2d(2.719, 5.335), Rotation2d.fromDegrees(0)))
    //         .withTimeout(3));

    // NamedCommands.registerCommand(
    // "AutoScore",
    // ShooterCommands.shootAutoAim(shooter, intake, floor, feeder, drive).withTimeout(5));
    NamedCommands.registerCommand(
        "AutoScore",
        shooter
            .setHoodPosition(Degree.of(5))
            .andThen(shooter.setShooterClosedLoop(RPM.of(535 * 5)))
            .alongWith(Commands.waitSeconds(1.25).andThen(feeder.setClosedLoop(RPM.of(5000))))
            .alongWith(floor.setClosedLoop(RPM.of(5000)))
            .alongWith(
                Commands.waitSeconds(0.75)
                    .andThen(intake.setDeployOpenLoop(Volts.of(4)).withTimeout(1.5)))
            .withTimeout(3.25)
            .andThen(shooter.setHoodPosition(Degree.of(0))));

    NamedCommands.registerCommand(
        "AutoScore12",
        shooter
            .setShooterClosedLoopAndAngle(RPM.of(510 * 5), Degrees.of(20))
            .alongWith(Commands.waitSeconds(1).andThen(feeder.setClosedLoop(RPM.of(4000))))
            .alongWith(floor.setClosedLoop(RPM.of(4000)))
            .alongWith(
                Commands.waitSeconds(0.75)
                    .andThen(intake.setDeployOpenLoop(Volts.of(2)).withTimeout(1.5)))
            .withTimeout(3.25));

    NamedCommands.registerCommand(
        "Volley",
        shooter
            .setShooterClosedLoopAndAngle(RPM.of(510 * 5), Degrees.of(20))
            .alongWith(Commands.waitSeconds(1).andThen(feeder.setClosedLoop(RPM.of(4000))))
            .alongWith(floor.setClosedLoop(RPM.of(4000)))
            .alongWith(
                Commands.waitSeconds(0.75)
                    .andThen(intake.setDeployOpenLoop(Volts.of(2)).withTimeout(1.5)))
            .withTimeout(3.25));

    NamedCommands.registerCommand(
        "AngleToHub",
        SnapCommands.snapToAngle(
                drive,
                () -> {
                  Translation2d hubToRobot =
                      SnapCommands.getHubCenter().minus(drive.getPose().getTranslation());
                  double angleToRobot = -Math.atan2(hubToRobot.getY(), -hubToRobot.getX());
                  return new Rotation2d(angleToRobot);
                })
            .withTimeout(1));

    NamedCommands.registerCommand(
        "Deploy", intake.setDeployOpenLoopWithSpin(Volts.of(-6), RPM.of(7000)).withTimeout(1.2));

    NamedCommands.registerCommand(
        "SnapToHub", SnapCommands.snapToRadius(drive, Feet.of(7.5)).withTimeout(2));
    NamedCommands.registerCommand(
        "SnapToHub12", SnapCommands.snapToRadius(drive, Feet.of(11.5)).withTimeout(3));

    NamedCommands.registerCommand(
        "getTheBallsIntoTheHub",
        EverythingCommands.getTheBallsIntoTheHub(
                drive, shooter, floor, intake, feeder, () -> 0, () -> 0)
            .withTimeout(4));

    // NamedCommands.registerCommand(
    // "AutoShoot", RollerCommands.shootOpenLoop(shooter, floor, feeder,
    // intake).withTimeout(3));j

    // NamedCommands.registerCommand("SnapToRadius", DriveCommands.snapToRadius(drive,
    // Feet.of(10.0)));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addOption(
        "Center-Hub",
        RollerCommands.shootClosedLoop(
                shooter, floor, feeder, RPM.of(515 * 5), RPM.of(1000), RPM.of(1500))
            .withTimeout(17));

    autoChooser.addOption(
        "Angle-Hub",
        RollerCommands.shootClosedLoop(
                shooter, floor, feeder, RPM.of(545 * 5), RPM.of(1000), RPM.of(1500))
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
    // ShooterCommands.shooterDefaultCommand(
    // shooter,
    // () -> SnapCommands.distanceToHub(drive),
    // () -> -driverController.getRightY(),
    // Feet.of(5)));
    intake.setDefaultCommand(
        intake.setJoystickOpenLoop(() -> -operatorController.getLeftX() * .85));
    feeder.setDefaultCommand(
        feeder.setJoystickOpenLoop(() -> -operatorController.getLeftY() * .85));
    floor.setDefaultCommand(floor.setJoystickOpenLoop(() -> -operatorController.getRightX() * .85));
    shooter.setDefaultCommand(
        ShooterCommands.shooterAllianceSideDefaultCommand(
            shooter, drive, () -> -operatorController.getRightY() * .85));
    leds.setDefaultCommand(LEDCommands.defaultCommand(leds, vision::hasSeenAprilTag));

    // =========================================
    // ============ Driver Controls ============
    // =========================================

    // Find somewhere to put this: .whileTrue(RollerCommands.intakeJiggleOpenLoop(intake));

    // Reset gyro to 0when B button is pressed
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

    driverController.leftTrigger().whileTrue(intake.setIntakeClosedLoop(RPM.of(7000)));

    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    // // driverController.a().whileTrue(SnapCommands.snapToRadius(drive, Feet.of(7.5)));
    // // driverController.b().whileTrue(RollerCommands.intakeJiggleOpenLoop(intake));
    // // driverController.y().whileTrue(SnapCommands.snapToRadius(drive, Feet.of(11.5)));
    // driverController.y().whileTrue(SnapCommands.tuneableSnapToRadius(drive));

    // driverController.b().whileTrue(intake.setDeployOpenLoop(Volts.of(-8)));
    // driverController.x().whileTrue(SnapCommands.tuneableSnapToRadius(drive));

    // driverController
    //     .rightTrigger()
    //     .whileTrue(
    //         shooter
    //             .setTunableShootWithHood()
    //
    // .alongWith(Commands.waitSeconds(1.25).andThen(feeder.setClosedLoop(RPM.of(4000))))
    //             .alongWith(floor.setClosedLoop(RPM.of(4000))));

    // =========================================
    // =========== Operator Controls ===========
    // =========================================

    driverController.b().whileTrue(intake.setDeployOpenLoop(Volts.of(-4)));
    driverController.x().whileTrue(intake.setDeployOpenLoop(Volts.of(4)));

    driverController
        .y()
        .whileTrue(
            EverythingCommands.getTheBallsIntoTheHub(
                    drive,
                    shooter,
                    floor,
                    intake,
                    feeder,
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX())
                .alongWith(ControllerCommands.rumble(driverController)));

    driverController
        .rightBumper()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    () -> {
                      Translation2d vollyToRobot =
                          SnapCommands.getRightVolley().minus(drive.getPose().getTranslation());
                      double angleToRobot =
                          Math.atan2(vollyToRobot.getY(), vollyToRobot.getX()) + Math.PI;
                      return new Rotation2d(angleToRobot);
                    })
                .alongWith(ControllerCommands.rumble(operatorController))
                .alongWith(
                    ShooterCommands.shootAutoAimVollyContinuous(shooter, floor, feeder, drive))
                .alongWith(
                    Commands.waitSeconds(1.75)
                        .andThen(RollerCommands.intakeJiggleOpenLoop(intake))));

    driverController
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -driverController.getLeftY(),
                    () -> -driverController.getLeftX(),
                    () -> {
                      Translation2d vollyToRobot =
                          SnapCommands.getLeftVolley().minus(drive.getPose().getTranslation());
                      double angleToRobot =
                          Math.atan2(vollyToRobot.getY(), vollyToRobot.getX()) + Math.PI;
                      return new Rotation2d(angleToRobot);
                    })
                .alongWith(ControllerCommands.rumble(operatorController))
                .alongWith(
                    ShooterCommands.shootAutoAimVollyContinuous(shooter, floor, feeder, drive))
                .alongWith(
                    Commands.waitSeconds(1.75)
                        .andThen(RollerCommands.intakeJiggleOpenLoop(intake))));

    operatorController.leftTrigger().whileTrue(intake.setIntakeClosedLoop(RPM.of(7000)));

    // // operatorController
    // //     .y()
    // //     .whileTrue(
    // //         shooter
    // //             .setShooterClosedLoopAndAngle(RPM.of(510), Degrees.of(20))
    // //
    // // .alongWith(Commands.waitSeconds(1.25).andThen(feeder.setClosedLoop(RPM.of(4000))))
    // //             .alongWith(floor.setClosedLoop(RPM.of(4000)))
    // //             .alongWith(ControllerCommands.rumble(driverController)));

    // operatorController
    //     .y()
    //     .whileTrue(ShooterCommands.shootAutoAimContinuous(shooter, floor, feeder, drive));
    // // 20 degree angle

    // // operatorController
    // //     .a()
    // //

    // operatorController.a().whileTrue(ShooterCommands.shootAutoAim(shooter, floor, feeder,
    // drive));

    // // operatorController
    // //     .y()
    // //     .whileTrue(ShooterCommands.shootAutoAimContinuous(shooter, floor, feeder, drive));

    // operatorController
    //     .povUp()
    //     .whileTrue(
    //         shooter
    //             .setShooterClosedLoopAndAngle(RPM.of(600), Degrees.of(30))
    //
    // .alongWith(Commands.waitSeconds(0.75).andThen(feeder.setClosedLoop(RPM.of(4000))))
    //             .alongWith(floor.setClosedLoop(RPM.of(4000)))
    //             .alongWith(ControllerCommands.rumble(driverController)));

    // // operatorController
    // //     .start()
    // //     .whileTrue(
    // //         shooter
    // //             .setShooterClosedLoop(RPM.of(445))
    // //
    // // .alongWith(Commands.waitSeconds(1.25).andThen(feeder.setClosedLoop(RPM.of(4000))))
    // //             .alongWith(floor.setClosedLoop(RPM.of(4000))));

    // // operatorController.povUp().onTrue(shooter.incrementSetHoodPosition(Degrees.of(15)));

    // // operatorController.povDown().onTrue(shooter.incrementSetHoodPosition(Degrees.of(-15)));

    // // operatorController
    // //     .rightBumper()
    // //     .onTrue(
    // //         Commands.runOnce(
    // //             () -> {
    // //               ShooterConstants.Real.shooterSpeed.incrementBy(15);
    // //             }));
    // // operatorController
    // //     .leftBumper()
    // //     .onTrue(
    // //         Commands.runOnce(
    // //             () -> {
    // //               ShooterConstants.Real.shooterSpeed.incrementBy(-15);
    // //             }));

    // // operatorController
    // //     .rightTrigger()
    // //     .whileTrue(
    // //         RollerCommands.tuneableShootClosedLoop(
    // //                 shooter, floor, feeder, RPM.of(1000), RPM.of(2000))
    // //             .alongWith(ControllerCommands.rumble(driverController)));

    // // =========================================
    // // ============ Jithin Controls ============
    // // =========================================

    // // jithinController
    // //     .a()
    // //     .whileTrue(
    // //         shooter
    // //             .setShooterClosedLoop(RPM.of(420))
    // //
    // // .alongWith(Commands.waitSeconds(1.25).andThen(feeder.setClosedLoop(RPM.of(4000))))
    // //             .alongWith(floor.setClosedLoop(RPM.of(4000))));

    // jithinController
    //     .b()
    //     .whileTrue(
    //         shooter
    //             .setShooterClosedLoop(RPM.of(440))
    //
    // .alongWith(Commands.waitSeconds(1.25).andThen(feeder.setClosedLoop(RPM.of(4000))))
    //             .alongWith(floor.setClosedLoop(RPM.of(4000))));

    // jithinController
    //     .x()
    //     .whileTrue(
    //         shooter
    //             .setShooterClosedLoop(RPM.of(450))
    //
    // .alongWith(Commands.waitSeconds(1.25).andThen(feeder.setClosedLoop(RPM.of(4000))))
    //             .alongWith(floor.setClosedLoop(RPM.of(4000))));

    jithinController.y().whileTrue(SnapCommands.snapToRadius(drive, Feet.of(11.5)));

    jithinController.leftTrigger().whileTrue(intake.setTunableIntake());
    jithinController.leftBumper().whileTrue(floor.setTunableFloor());
    jithinController.rightTrigger().whileTrue(shooter.setTunableShooter());
    jithinController.rightBumper().whileTrue(feeder.setTunableFeeder());

    jithinController.povUp().onTrue(shooter.incrementSetHoodPosition(Degrees.of(15)));

    jithinController.povDown().onTrue(shooter.incrementSetHoodPosition(Degrees.of(-15)));

    // // jithinController.povUp().whileTrue(intake.setTunableIntakeDeploy());
    // // jithinController.povDown().whileTrue(intake.setTunableIntakeDeployBack());
    // jithinController.povUp().onTrue(shooter.incrementSetHoodPosition(Degrees.of(5)));
    // jithinController.povDown().onTrue(shooter.incrementSetHoodPosition(Degrees.of(-5)));
    jithinController.povLeft().whileTrue(intake.setOpenLoop(Volts.of(5)));
    // jithinController.povRight().whileTrue(intake.setDeployOpenLoop(Volts.of(3)));

    // // jithinController.povDown().onTrue(intake.setDeployOpenLoop(Volts.of(5)).withTimeout(.5));

    jithinController
        .a()
        .whileTrue(
            shooter
                .setShooterClosedLoopAndAngle(RPM.of(510), Degrees.of(20))
                .alongWith(Commands.waitSeconds(1).andThen(feeder.setClosedLoop(RPM.of(6000))))
                .alongWith(floor.setClosedLoop(RPM.of(6000)))
                .alongWith(
                    Commands.waitSeconds(1.0)
                        .andThen(intake.setDeployOpenLoop(Volts.of(3)).withTimeout(2.0))));

    // jithinController.y().whileTrue(RollerCommands.intakeJiggleOpenLoop(intake));

    // jithinController
    //     .b()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -jithinController.getLeftY(),
    //             () -> -jithinController.getLeftX(),
    //             () -> {
    //               Translation2d hubToRobot =
    //                   SnapCommands.getHubCenter().minus(drive.getPose().getTranslation());
    //               double angleToRobot = Math.atan2(hubToRobot.getY(), hubToRobot.getX());
    //               return new Rotation2d(angleToRobot);
    //             }));
  }

  private boolean hasBeenInTeleop = false;

  @Override
  public void teleopInit() {
    // drive.swerveBreak(true);
    hasBeenInTeleop = true;
    LimelightHelpers.setRewindEnabled(VisionConstants.limelightFront, true);
  }

  @Override
  public void simulationInit() {
    if (!(SimulatedArena.getInstance() instanceof Arena2026Rebuilt arena)) return;

    arena.getBlueHub().setOnScoredCallback((gp) -> System.out.println("Blue Hub Scored!"));
    arena.getRedHub().setOnScoredCallback((gp) -> System.out.println("Red Hub Scored!"));
    arena.setNeutralFuelCount(80); // for performance
  }

  @Override
  public void autonomousInit() {
    LimelightHelpers.setRewindEnabled(VisionConstants.limelightFront, true);
  }

  @Override
  public void disabledInit() {
    boolean isFMSAttached = DriverStation.isFMSAttached();
    if (isFMSAttached && hasBeenInTeleop) {
      Logger.recordOutput(
          "Limelight/LimelightRewind",
          "Getting limelight rewind capture (fms="
              + isFMSAttached
              + ", hasBeenInTeleop="
              + hasBeenInTeleop
              + ")");
      LimelightHelpers.triggerRewindCapture(VisionConstants.limelightFront, 150);
    } else {
      Logger.recordOutput(
          "Limelight/LimelightRewind",
          "Skipping limelight rewind capture (fms="
              + isFMSAttached
              + ", hasBeenInTeleop="
              + hasBeenInTeleop
              + ")");
    }
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
    return Commands.sequence(
        Commands.print("=== System Tests Starting ==="),

        // Shooter
        Commands.print("Testing shooter..."),
        shooter.setShooterClosedLoopAndAngle(RPM.of(100), Degrees.of(14)).withTimeout(2),
        Commands.print("Shooter OK"),
        Commands.waitSeconds(0.5),

        // Feeder
        Commands.print("Testing feeder..."),
        feeder.setOpenLoop(Volts.of(3)).withTimeout(2),
        Commands.print("Feeder OK"),
        Commands.waitSeconds(0.5),

        // Floor
        Commands.print("Testing floor..."),
        floor.setOpenLoop(Volts.of(3)).withTimeout(2),
        Commands.print("Floor OK"),
        Commands.waitSeconds(0.5),

        // Intake roller
        Commands.print("Testing intake roller..."),
        intake.setIntakeClosedLoop(RPM.of(3000)).withTimeout(2),
        Commands.print("Intake OK"),
        Commands.waitSeconds(0.5),

        // Intake deploy
        Commands.print("Testing intake deploy..."),
        intake.setDeployOpenLoop(Volts.of(4)).withTimeout(1),
        Commands.waitSeconds(0.1),
        intake.setDeployOpenLoop(Volts.of(-4)).withTimeout(1),
        Commands.print("Intake deploy OK"),
        Commands.print("=== All tests Ran ==="));
  }
}
