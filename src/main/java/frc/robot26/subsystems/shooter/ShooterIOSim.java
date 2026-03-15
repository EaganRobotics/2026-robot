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
  private static final DCMotor shooterGearbox = DCMotor.getKrakenX60(4);
  private final SimulatedMotorController.GenericMotorController shooterMotorController;
  private final MapleMotorSim shooterMotor;
  private Voltage shooterAppliedVoltage = Volts.of(0);

  private static final DCMotor hoodGearbox = DCMotor.getKrakenX44(1);
  private final SimulatedMotorController.GenericMotorController hoodMotorController;
  private final MapleMotorSim hoodMotor;
  private Voltage hoodAppliedVoltage = Volts.of(0);

  private static final double LAUNCH_VOLTAGE_THRESHOLD = 1.0; // volts
  private boolean wasAboveThreshold = false;

  /** Stored each updateInputs so simulateProjectile can read current RPM. */
  private double currentShooterRPM = 0.0;

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

    // Store RPM for use in simulateProjectile
    currentShooterRPM =
        AngularVelocity.ofBaseUnits(shooterAngularVelocity, RadiansPerSecond).in(RPM);
  }

  /**
   * Call this from RobotContainer.simulationPeriodic() to launch a fuel projectile when the shooter
   * spins up past the threshold.
   */
  public void simulateProjectile(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    boolean aboveThreshold = Math.abs(shooterAppliedVoltage.in(Volts)) > LAUNCH_VOLTAGE_THRESHOLD;

    // Launch on rising edge only (once per spin-up)
    if (aboveThreshold && !wasAboveThreshold) {
      RebuiltFuelOnFly fuel =
          new RebuiltFuelOnFly(
              robotPose.getTranslation(),
              new Translation2d(0.2, 0),
              fieldRelativeSpeeds,
              robotPose.getRotation(),
              Meters.of(0.45),
              MetersPerSecond.of(Math.abs(shooterAppliedVoltage.in(Volts)) / 12.0 * 20.0),
              Radians.of(Math.toRadians(55)));

      // All chained methods return GamePieceProjectile, so configure on the base type
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
    }

    wasAboveThreshold = aboveThreshold;
  }
}
