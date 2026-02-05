package frc.robot26.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot26.subsystems.feeder.FeederConstants.GEARING;
import static frc.robot26.subsystems.feeder.FeederConstants.SUPPLY_CURRENT_LIMIT;
import static frc.robot26.subsystems.feeder.FeederConstants.Sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class FeederIOSim implements FeederIO {
  private static final DCMotor feederGearbox = DCMotor.getKrakenX44(2);
  private final SimulatedMotorController.GenericMotorController feederMotorController;
  private final MapleMotorSim feederMotor;
  private Voltage feederAppliedVoltage = Volts.of(0);

  // one is the actual simulator and one is like the which model is used and its
  // gearbox configuration, using both flywheelsim and maple motor sim is good
  private final FlywheelSim feederSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(feederGearbox, 0.1, GEARING),
          feederGearbox,
          0.000015);

  public FeederIOSim() {
    feederMotor =
        new MapleMotorSim(
            new SimMotorConfigs(feederGearbox, GEARING, Sim.MOTOR_LOAD_MOI, Sim.FRICTION_VOLTAGE));
    feederMotorController =
        feederMotor.useSimpleDCMotorController().withCurrentLimit(SUPPLY_CURRENT_LIMIT);
  }

  @Override
  public void setFeederOpenLoop(Voltage output) {
    feederAppliedVoltage = output;
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    var feederAngularVelocity = feederSim.getAngularVelocityRadPerSec();

    feederMotorController.requestVoltage(feederAppliedVoltage);
    feederSim.setInputVoltage(feederMotor.getAppliedVoltage().in(Volts));
    feederMotor.update(Seconds.of(TimedRobot.kDefaultPeriod));
    feederSim.update(TimedRobot.kDefaultPeriod);

    // Update motor inputs
    inputs.feederConnected = true;
    inputs.feederAppliedVolts = feederAppliedVoltage;
    inputs.feederCurrent = Amps.of(feederSim.getCurrentDrawAmps());
    inputs.feederVelocity = AngularVelocity.ofBaseUnits(feederAngularVelocity, RadiansPerSecond);
  }
}
