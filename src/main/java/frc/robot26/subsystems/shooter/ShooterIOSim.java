package frc.robot26.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot26.subsystems.shooter.ShooterConstants.GEARING_HOOD;
import static frc.robot26.subsystems.shooter.ShooterConstants.GEARING_SHOOTER;
import static frc.robot26.subsystems.shooter.ShooterConstants.SUPPLY_CURRENT_LIMIT;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot26.subsystems.shooter.ShooterConstants.Sim;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class ShooterIOSim implements ShooterIO {
  private static final DCMotor shooterGearbox = DCMotor.getKrakenX60(4);
  private final SimulatedMotorController.GenericMotorController shooterMotorController;
  private final MapleMotorSim shooterMotor;
  private Voltage shooterAppliedVoltage = Volts.of(0);

  private static final DCMotor hoodGearbox = DCMotor.getKrakenX44(1);
  private final SimulatedMotorController.GenericMotorController hoodMotorController;
  private final MapleMotorSim hoodMotor;
  private Voltage hoodAppliedVoltage = Volts.of(0);

  // one is the actual simulator and one is like the which model is used and its
  // gearbox configuration, using both flywheelsim and maple motor sim is good
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
    shooterAppliedVoltage =
        Volts.of(velocity.in(RPM) * 0.01); // Convert RPM to voltage (simplified)
    setShooterOpenLoop(shooterAppliedVoltage); // Use the open loop method to set the voltage
  }

  @Override
  public void setShooterClosedLoop(AngularVelocity velocity, double accelerationLimitRpmPerSecond) {
    setShooterClosedLoop(velocity);
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

    // Update motor inputs
    inputs.shooterConnected = true;
    inputs.shooterAppliedVolts = shooterAppliedVoltage;
    inputs.shooterCurrent = Amps.of(shooterSim.getCurrentDrawAmps());
    inputs.shooterVelocity = AngularVelocity.ofBaseUnits(shooterAngularVelocity, RadiansPerSecond);

    inputs.hoodConnected = true;
    inputs.hoodAppliedVolts = hoodAppliedVoltage;
    inputs.hoodCurrent = Amps.of(hoodSim.getCurrentDrawAmps());
    inputs.hoodVelocity = AngularVelocity.ofBaseUnits(hoodAngularVelocity, RadiansPerSecond);
  }
}
