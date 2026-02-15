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
  private static final DCMotor climberGearbox = DCMotor.getKrakenX44(2);
  private final SimulatedMotorController.GenericMotorController climberMotorController;
  private final MapleMotorSim climberMotor;
  private Voltage climberAppliedVoltage = Volts.of(0);

  // one is the actual simulator and one is like the which model is used and its
  // gearbox configuration, using both flywheelsim and maple motor sim is good
  private final FlywheelSim ClimberSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(climberGearbox, 0.1, GEARING),
          climberGearbox,
          0.000015);

  public ClimberIOSim() {
    climberMotor =
        new MapleMotorSim(
            new SimMotorConfigs(climberGearbox, GEARING, Sim.MOTOR_LOAD_MOI, Sim.FRICTION_VOLTAGE));
    climberMotorController =
        climberMotor.useSimpleDCMotorController().withCurrentLimit(SUPPLY_CURRENT_LIMIT);
  }

  @Override
  public void setClimberOpenLoop(Voltage output) {
    climberAppliedVoltage = output;
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    var ClimberAngularVelocity = ClimberSim.getAngularVelocityRadPerSec();

    climberMotorController.requestVoltage(climberAppliedVoltage);
    ClimberSim.setInputVoltage(climberMotor.getAppliedVoltage().in(Volts));
    climberMotor.update(Seconds.of(TimedRobot.kDefaultPeriod));
    ClimberSim.update(TimedRobot.kDefaultPeriod);

    // Update motor inputs
    inputs.climberConnected = true;
    inputs.climberAppliedVolts = climberAppliedVoltage;
    inputs.climberCurrent = Amps.of(ClimberSim.getCurrentDrawAmps());
    inputs.climberVelocity = AngularVelocity.ofBaseUnits(ClimberAngularVelocity, RadiansPerSecond);
  }
}
