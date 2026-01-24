package frc.robot26.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot26.subsystems.intake.IntakeConstants.GEARING;
import static frc.robot26.subsystems.intake.IntakeConstants.SUPPLY_CURRENT_LIMIT;
import static frc.robot26.subsystems.intake.IntakeConstants.Sim;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot26.subsystems.intake.IntakeConstants.Sim;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class IntakeIOSim implements IntakeIO {
  // TODO: change to correct motor
  private static final DCMotor intakeGearbox = DCMotor.getKrakenX60(2);
  private final SimulatedMotorController.GenericMotorController intakeMotorController;
  private final MapleMotorSim intakeMotor;
  private Voltage intakeAppliedVoltage = Volts.of(0);
  private final SimpleMotorFeedforward feedForwardController =
      new SimpleMotorFeedforward(Sim.kS, Sim.kV, Sim.kA);

  private final FlywheelSim intakeSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(intakeGearbox, 0.1, GEARING),
          intakeGearbox,
          0.000015);

  public IntakeIOSim() {
    intakeMotor =
        new MapleMotorSim(
            new SimMotorConfigs(intakeGearbox, GEARING, Sim.MOTOR_LOAD_MOI, Sim.FRICTION_VOLTAGE));
    intakeMotorController =
        intakeMotor.useSimpleDCMotorController().withCurrentLimit(SUPPLY_CURRENT_LIMIT);
  }

  @Override
  public void setOpenLoop(Voltage output) {
    intakeAppliedVoltage = output;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var angularVelocity = intakeSim.getAngularVelocityRadPerSec();

    intakeMotorController.requestVoltage(intakeAppliedVoltage);
    intakeSim.setInputVoltage(intakeMotor.getAppliedVoltage().in(Volts));
    intakeMotor.update(Seconds.of(TimedRobot.kDefaultPeriod));
    intakeSim.update(TimedRobot.kDefaultPeriod);

    // Update motor inputs
    inputs.intakeConnected = true;
    inputs.intakeAppliedVolts = intakeAppliedVoltage;
    inputs.intakeCurrent = Amps.of(intakeSim.getCurrentDrawAmps());
    inputs.intakeVelocity = AngularVelocity.ofBaseUnits(angularVelocity, RadiansPerSecond);
  }
}
