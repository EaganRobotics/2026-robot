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
  private static final double LAUNCH_VOLTAGE_THRESHOLD = 1.0; // volts
  private static final double FIRE_RATE_SECONDS = 0.1;
  private static final double MAX_LAUNCH_SPEED_MPS = 20.0;
  private static final double LAUNCH_ANGLE_DEGREES = 55.0;
  private static final double LAUNCH_HEIGHT_METERS = 0.45;
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

  private double currentShooterRPM = 0.0;
  private double timeSinceLastShot = FIRE_RATE_SECONDS; // ready to fire immediately

  public double getCurrentShooterRPM() {
    return currentShooterRPM;
  }

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
    shooterMotorController.requestVoltage(shooterAppliedVoltage);
    shooterSim.setInputVoltage(shooterMotor.getAppliedVoltage().in(Volts));
    shooterMotor.update(Seconds.of(TimedRobot.kDefaultPeriod));
    shooterSim.update(TimedRobot.kDefaultPeriod);

    hoodMotorController.requestVoltage(hoodAppliedVoltage);
    hoodSim.setInputVoltage(hoodMotor.getAppliedVoltage().in(Volts));
    hoodMotor.update(Seconds.of(TimedRobot.kDefaultPeriod));
    hoodSim.update(TimedRobot.kDefaultPeriod);

    var shooterAngularVelocity = shooterSim.getAngularVelocityRadPerSec();
    var hoodAngularVelocity = hoodSim.getAngularVelocityRadPerSec();

    inputs.shooterConnected = true;
    inputs.shooterAppliedVolts = shooterAppliedVoltage;
    inputs.shooterCurrent = Amps.of(shooterSim.getCurrentDrawAmps());
    inputs.shooterVelocity = AngularVelocity.ofBaseUnits(shooterAngularVelocity, RadiansPerSecond);

    inputs.hoodConnected = true;
    inputs.hoodAppliedVolts = hoodAppliedVoltage;
    inputs.hoodCurrent = Amps.of(hoodSim.getCurrentDrawAmps());
    inputs.hoodVelocity = AngularVelocity.ofBaseUnits(hoodAngularVelocity, RadiansPerSecond);

    currentShooterRPM =
        AngularVelocity.ofBaseUnits(shooterAngularVelocity, RadiansPerSecond).in(RPM);
  }

  /**
   * Call this from RobotContainer.simulationPeriodic() to launch fuel projectiles while voltage is
   * applied — but only if there are balls stored in the robot.
   */
  public void simulateProjectile(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    boolean shooting = Math.abs(shooterAppliedVoltage.in(Volts)) > LAUNCH_VOLTAGE_THRESHOLD;

    if (shooting && RobotGamePieceStorage.hasBalls()) {
      timeSinceLastShot += TimedRobot.kDefaultPeriod;

      if (timeSinceLastShot >= FIRE_RATE_SECONDS) {
        timeSinceLastShot = 0.0;

        RebuiltFuelOnFly fuel =
            new RebuiltFuelOnFly(
                robotPose.getTranslation(),
                SHOOTER_OFFSET,
                fieldRelativeSpeeds,
                robotPose.getRotation(),
                Meters.of(LAUNCH_HEIGHT_METERS),
                MetersPerSecond.of(
                    Math.abs(shooterAppliedVoltage.in(Volts)) / 12.0 * MAX_LAUNCH_SPEED_MPS),
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

        Logger.recordOutput("Shooter/BallsRemaining", RobotGamePieceStorage.getStoredBalls());
      }
    } else if (!shooting) {
      // Reset timer when not shooting so first shot fires immediately next time
      timeSinceLastShot = FIRE_RATE_SECONDS;
    }
  }
}
