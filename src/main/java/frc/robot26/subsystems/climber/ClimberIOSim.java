package frc.robot26.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot26.subsystems.feeder.FeederConstants.GEARING;
import static frc.robot26.subsystems.feeder.FeederConstants.SUPPLY_CURRENT_LIMIT;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot26.subsystems.climber.ClimberConstants.Sim;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class ClimberIOSim implements ClimberIO {
  private static final DCMotor ClimberGearbox = DCMotor.getKrakenX44(2);
  private final SimulatedMotorController.GenericMotorController ClimberMotorController;
  private final MapleMotorSim ClimberMotor;
  private Voltage ClimberAppliedVoltage = Volts.of(0);

  // one is the actual simulator and one is like the which model is used and its
  // gearbox configuration, using both flywheelsim and maple motor sim is good
  private final FlywheelSim ClimberSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(ClimberGearbox, 0.1, GEARING),
          ClimberGearbox,
          0.000015);

  public ClimberIOSim() {
    ClimberMotor =
        new MapleMotorSim(
            new SimMotorConfigs(ClimberGearbox, GEARING, Sim.MOTOR_LOAD_MOI, Sim.FRICTION_VOLTAGE));
    ClimberMotorController =
        ClimberMotor.useSimpleDCMotorController().withCurrentLimit(SUPPLY_CURRENT_LIMIT);
  }

  @Override
  public void setClimberOpenLoop(Voltage output) {
    ClimberAppliedVoltage = output;
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    var ClimberAngularVelocity = ClimberSim.getAngularVelocityRadPerSec();

    ClimberMotorController.requestVoltage(ClimberAppliedVoltage);
    ClimberSim.setInputVoltage(ClimberMotor.getAppliedVoltage().in(Volts));
    ClimberMotor.update(Seconds.of(TimedRobot.kDefaultPeriod));
    ClimberSim.update(TimedRobot.kDefaultPeriod);

    // Update motor inputs
    inputs.ClimberConnected = true;
    inputs.ClimberAppliedVolts = ClimberAppliedVoltage;
    inputs.ClimberCurrent = Amps.of(ClimberSim.getCurrentDrawAmps());
    inputs.ClimberVelocity = AngularVelocity.ofBaseUnits(ClimberAngularVelocity, RadiansPerSecond);
  }
}
