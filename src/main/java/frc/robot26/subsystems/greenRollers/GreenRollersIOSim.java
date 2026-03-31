package frc.robot26.subsystems.greenRollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot26.subsystems.greenRollers.GreenRollersConstants.GEARING;
import static frc.robot26.subsystems.greenRollers.GreenRollersConstants.SUPPLY_CURRENT_LIMIT;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot26.subsystems.greenRollers.GreenRollersConstants.Sim;
import frc.robot26.subsystems.greenRollers.GreenRollersIO.GreenRollersIOInputs;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class GreenRollersIOSim implements GreenRollersIO {
  private static final DCMotor greenRollersGearbox = DCMotor.getKrakenX44(1);
  private final SimulatedMotorController.GenericMotorController greenRollersMotorController;
  private final MapleMotorSim greenRollersMotor;
  private Voltage greenRollersAppliedVoltage = Volts.of(0);

  // one is the actual simulator and one is like the which model is used and its
  // gearbox configuration, using both flywheelsim and maple motor sim is good
  private final FlywheelSim greenRollersSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(greenRollersGearbox, 0.1, GEARING),
          greenRollersGearbox,
          0.000015);

  public GreenRollersIOSim() {
    greenRollersMotor =
        new MapleMotorSim(
            new SimMotorConfigs(
                greenRollersGearbox, GEARING, Sim.MOTOR_LOAD_MOI, Sim.FRICTION_VOLTAGE));
    greenRollersMotorController =
        greenRollersMotor.useSimpleDCMotorController().withCurrentLimit(SUPPLY_CURRENT_LIMIT);
  }

  @Override
  public void setGreenRollersOpenLoop(Voltage output) {
    greenRollersAppliedVoltage = output;
  }

  @Override
  public void setGreenRollersClosedLoop(AngularVelocity velocity) {
    greenRollersAppliedVoltage =
        Volts.of(velocity.in(RPM) * 0.01); // Convert RPM to voltage (simplified)
    setGreenRollersOpenLoop(greenRollersAppliedVoltage);
  }

  @Override
  public void updateInputs(GreenRollersIOInputs inputs) {
    greenRollersMotorController.requestVoltage(greenRollersAppliedVoltage);
    greenRollersSim.setInputVoltage(greenRollersMotor.getAppliedVoltage().in(Volts));
    greenRollersMotor.update(Seconds.of(TimedRobot.kDefaultPeriod));
    greenRollersSim.update(TimedRobot.kDefaultPeriod);
    var greenRollersAngularVelocity = greenRollersSim.getAngularVelocityRadPerSec();

    // Update motor inputs
    inputs.greenRollersConnected = true;
    inputs.greenRollersAppliedVolts = greenRollersAppliedVoltage;
    inputs.greenRollersCurrent = Amps.of(greenRollersSim.getCurrentDrawAmps());
    inputs.greenRollersVelocity =
        AngularVelocity.ofBaseUnits(greenRollersAngularVelocity, RadiansPerSecond);
  }
}
