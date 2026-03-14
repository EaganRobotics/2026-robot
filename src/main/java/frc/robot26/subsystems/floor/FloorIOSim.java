package frc.robot26.subsystems.floor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot26.subsystems.floor.FloorConstants.GEARING;
import static frc.robot26.subsystems.floor.FloorConstants.SUPPLY_CURRENT_LIMIT;
import static frc.robot26.subsystems.floor.FloorConstants.Sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot26.subsystems.floor.FloorConstants.Sim;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class FloorIOSim implements FloorIO {
  private static final DCMotor floorGearbox = DCMotor.getKrakenX44(1);
  private final SimulatedMotorController.GenericMotorController floorMotorController;
  private final MapleMotorSim floorMotor;
  private Voltage floorAppliedVoltage = Volts.of(0);

  // one is the actual simulator and one is like the which model is used and its
  // gearbox configuration, using both flywheelsim and maple motor sim is good
  private final FlywheelSim floorSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(floorGearbox, 0.1, GEARING), floorGearbox, 0.000015);

  public FloorIOSim() {
    floorMotor =
        new MapleMotorSim(
            new SimMotorConfigs(floorGearbox, GEARING, Sim.MOTOR_LOAD_MOI, Sim.FRICTION_VOLTAGE));
    floorMotorController =
        floorMotor.useSimpleDCMotorController().withCurrentLimit(SUPPLY_CURRENT_LIMIT);
  }

  @Override
  public void setFloorOpenLoop(Voltage output) {
    floorAppliedVoltage = output;
  }

  @Override
  public void setFloorClosedLoop(AngularVelocity velocity) {
    floorAppliedVoltage = Volts.of(velocity.in(RPM) * 0.01); // Convert RPM to voltage (simplified)
    setFloorOpenLoop(floorAppliedVoltage);
  }

  @Override
  public void updateInputs(FloorIOInputs inputs) {
    floorMotorController.requestVoltage(floorAppliedVoltage);
    floorSim.setInputVoltage(floorMotor.getAppliedVoltage().in(Volts));
    floorMotor.update(Seconds.of(TimedRobot.kDefaultPeriod));
    floorSim.update(TimedRobot.kDefaultPeriod);

    var floorAngularVelocity = floorSim.getAngularVelocityRadPerSec();

    // Update motor inputs
    inputs.floorConnected = true;
    inputs.floorAppliedVolts = floorAppliedVoltage;
    inputs.floorCurrent = Amps.of(floorSim.getCurrentDrawAmps());
    inputs.floorVelocity = AngularVelocity.ofBaseUnits(floorAngularVelocity, RadiansPerSecond);
  }
}
