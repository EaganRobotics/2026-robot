package frc.robot26.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot26.subsystems.shooter.ShooterConstants.GEARING_HOOD;
import static frc.robot26.subsystems.shooter.ShooterConstants.GEARING_SHOOTER;
import static frc.robot26.subsystems.shooter.ShooterConstants.SUPPLY_CURRENT_LIMIT;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot26.subsystems.intake.RobotGamePieceStorage;
import frc.robot26.subsystems.shooter.ShooterConstants.Sim;
import java.util.List;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public class ShooterIOSim implements ShooterIO {

  // ── Projectile tuning ─────────────────────────────────────────────────────
  /** Minimum RPM before projectiles will fire. */
  private static final double LAUNCH_RPM_THRESHOLD = 50.0;

  private static final double FIRE_RATE_SECONDS = 0.1;

  /** Launch speed in m/s per RPM. e.g. at 0.005: 1000 RPM -> 5 m/s, 6000 RPM -> 30 m/s */
  private static final double LAUNCH_SPEED_PER_RPM = 0.005; // m/s per RPM — TUNE THIS

  /** Hard cap on launch speed regardless of RPM. */
  private static final double MAX_LAUNCH_SPEED_MPS = 30.0;

  private static final double LAUNCH_ANGLE_DEGREES = 55.0;
  private static final double LAUNCH_HEIGHT_METERS = 0.45;

  /** Number of balls fired side-by-side per shot burst. */
  private static final int BALLS_WIDE = 4;

  /**
   * Total lateral spread across all balls in meters (robot-relative, perpendicular to facing). e.g.
   * 0.3 = 4 balls spanning 0.3 m → spaced 0.1 m apart.
   */
  private static final double SPREAD_WIDTH_METERS = 0.3;

  /** Forward offset of the shooter from robot center. */
  private static final Translation2d SHOOTER_OFFSET = new Translation2d(0.2, 0);

  // ── Motor setup ───────────────────────────────────────────────────────────
  private static final DCMotor shooterGearbox = DCMotor.getKrakenX60(4);
  private final SimulatedMotorController.GenericMotorController shooterMotorController;
  private final MapleMotorSim shooterMotor;
  private Voltage shooterAppliedVoltage = Volts.of(0);

  private static final DCMotor hoodGearbox = DCMotor.getKrakenX44(1);
  private final SimulatedMotorController.GenericMotorController hoodMotorController;
  private final MapleMotorSim hoodMotor;
  private Voltage hoodAppliedVoltage = Volts.of(0);

  private double timeSinceLastShot = FIRE_RATE_SECONDS; // ready to fire immediately

  // FlywheelSim is the source of truth for RPM — driven DIRECTLY from shooterAppliedVoltage
  private final FlywheelSim shooterSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(shooterGearbox, 0.1, GEARING_SHOOTER),
          shooterGearbox,
          0.000015);

  private final FlywheelSim hoodSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(hoodGearbox, 0.1, GEARING_HOOD),
          hoodGearbox,
          0.000015);

  public ShooterIOSim() {
    shooterMotor =
        new MapleMotorSim(
            new SimMotorConfigs(
                shooterGearbox, GEARING_SHOOTER, Sim.MOTOR_LOAD_MOI, Sim.FRICTION_VOLTAGE));
    shooterMotorController =
        shooterMotor.useSimpleDCMotorController().withCurrentLimit(SUPPLY_CURRENT_LIMIT);

    hoodMotor =
        new MapleMotorSim(
            new SimMotorConfigs(
                hoodGearbox, GEARING_HOOD, Sim.MOTOR_LOAD_MOI, Sim.FRICTION_VOLTAGE));
    hoodMotorController =
        hoodMotor.useSimpleDCMotorController().withCurrentLimit(SUPPLY_CURRENT_LIMIT);
  }

  @Override
  public void setShooterOpenLoop(Voltage output) {
    shooterAppliedVoltage = output;
  }

  @Override
  public void setHoodOpenLoop(Voltage output) {
    hoodAppliedVoltage = output;
  }

  @Override
  public void setShooterClosedLoop(AngularVelocity velocity) {
    shooterAppliedVoltage = Volts.of(velocity.in(RPM) * 0.01);
    setShooterOpenLoop(shooterAppliedVoltage);
  }

  @Override
  public void setHoodPosition(Angle angle) {
    hoodAppliedVoltage = Volts.of(angle.in(Degrees)); // this is wrong
    setHoodOpenLoop(hoodAppliedVoltage);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Drive FlywheelSim directly from our voltage command (clamped to ±12V)
    shooterSim.setInputVoltage(Math.max(-12.0, Math.min(12.0, shooterAppliedVoltage.in(Volts))));
    shooterSim.update(TimedRobot.kDefaultPeriod);

    hoodSim.setInputVoltage(Math.max(-12.0, Math.min(12.0, hoodAppliedVoltage.in(Volts))));
    hoodSim.update(TimedRobot.kDefaultPeriod);

    // Keep MapleMotorSim updated so it doesn't fall out of sync
    shooterMotorController.requestVoltage(shooterAppliedVoltage);
    shooterMotor.update(Seconds.of(TimedRobot.kDefaultPeriod));
    hoodMotorController.requestVoltage(hoodAppliedVoltage);
    hoodMotor.update(Seconds.of(TimedRobot.kDefaultPeriod));

    double shooterRPM =
        AngularVelocity.ofBaseUnits(shooterSim.getAngularVelocityRadPerSec(), RadiansPerSecond)
            .in(RPM);

    inputs.shooterConnected = true;
    inputs.shooterAppliedVolts = shooterAppliedVoltage;
    inputs.shooterCurrent = Amps.of(shooterSim.getCurrentDrawAmps());
    inputs.shooterVelocity =
        AngularVelocity.ofBaseUnits(shooterSim.getAngularVelocityRadPerSec(), RadiansPerSecond);

    inputs.hoodConnected = true;
    inputs.hoodAppliedVolts = hoodAppliedVoltage;
    inputs.hoodCurrent = Amps.of(hoodSim.getCurrentDrawAmps());
    inputs.hoodVelocity =
        AngularVelocity.ofBaseUnits(hoodSim.getAngularVelocityRadPerSec(), RadiansPerSecond);

    Logger.recordOutput("Shooter/SimRPM", shooterRPM);
    Logger.recordOutput("Shooter/SimVolts", shooterAppliedVoltage.in(Volts));
  }

  /**
   * Call from RobotContainer.simulationPeriodic(). Fires BALLS_WIDE projectiles side-by-side per
   * burst, spread SPREAD_WIDTH_METERS apart perpendicular to the robot's facing direction.
   */
  public void simulateProjectile(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    double currentRPM =
        AngularVelocity.ofBaseUnits(shooterSim.getAngularVelocityRadPerSec(), RadiansPerSecond)
            .in(RPM);
    boolean shooting = currentRPM > LAUNCH_RPM_THRESHOLD;

    if (shooting && RobotGamePieceStorage.hasBalls()) {
      timeSinceLastShot += TimedRobot.kDefaultPeriod;

      if (timeSinceLastShot >= FIRE_RATE_SECONDS) {
        timeSinceLastShot = 0.0;

        double launchSpeedMps = Math.min(currentRPM * LAUNCH_SPEED_PER_RPM, MAX_LAUNCH_SPEED_MPS);

        Logger.recordOutput("Shooter/LaunchRPM", currentRPM);
        Logger.recordOutput("Shooter/LaunchSpeedMps", launchSpeedMps);

        // Unit vectors for robot facing and the perpendicular (lateral) direction
        double robotAngle = robotPose.getRotation().getRadians();
        double lateralX = -Math.sin(robotAngle); // perpendicular to facing, field-relative X
        double lateralY = Math.cos(robotAngle); // perpendicular to facing, field-relative Y

        // Spread offsets: evenly distributed across SPREAD_WIDTH_METERS, centered at 0
        // e.g. BALLS_WIDE=4, SPREAD=0.3 -> offsets: -0.15, -0.05, +0.05, +0.15
        int ballsToFire = Math.min(BALLS_WIDE, RobotGamePieceStorage.getStoredBalls());
        for (int i = 0; i < ballsToFire; i++) {
          double t = (BALLS_WIDE == 1) ? 0.0 : (i / (double) (BALLS_WIDE - 1)) - 0.5;
          double lateralOffset = t * SPREAD_WIDTH_METERS;

          Translation2d perBallOffset =
              new Translation2d(
                  SHOOTER_OFFSET.getX() + lateralOffset * lateralX,
                  SHOOTER_OFFSET.getY() + lateralOffset * lateralY);

          RebuiltFuelOnFly fuel =
              new RebuiltFuelOnFly(
                  robotPose.getTranslation(),
                  perBallOffset,
                  fieldRelativeSpeeds,
                  robotPose.getRotation().plus(new edu.wpi.first.math.geometry.Rotation2d(Math.PI)),
                  Meters.of(LAUNCH_HEIGHT_METERS),
                  MetersPerSecond.of(launchSpeedMps),
                  Radians.of(Math.toRadians(LAUNCH_ANGLE_DEGREES)));

          GamePieceProjectile projectile = fuel;
          projectile
              .withTargetPosition(() -> new Translation3d(0.25, 5.56, 2.3))
              .withTargetTolerance(new Translation3d(0.5, 1.2, 0.3))
              .withHitTargetCallBack(() -> System.out.println("Fuel hit hub!"))
              .withProjectileTrajectoryDisplayCallBack(
                  (List<Pose3d> poses) ->
                      Logger.recordOutput("Shooter/ProjectileHit", poses.toArray(new Pose3d[0])),
                  (List<Pose3d> poses) ->
                      Logger.recordOutput("Shooter/ProjectileMiss", poses.toArray(new Pose3d[0])));

          SimulatedArena.getInstance().addGamePieceProjectile(projectile);
          RobotGamePieceStorage.removeBall();
        }

        Logger.recordOutput("Shooter/BallsRemaining", RobotGamePieceStorage.getStoredBalls());
      }
    } else if (!shooting) {
      timeSinceLastShot = FIRE_RATE_SECONDS;
    }
  }
}
