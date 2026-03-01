package frc.robot26.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot26.subsystems.intake.IntakeConstants.GEARING_DEPLOY;
import static frc.robot26.subsystems.intake.IntakeConstants.GEARING_INTAKE;
import static frc.robot26.subsystems.intake.IntakeConstants.SUPPLY_CURRENT_LIMIT;
import static frc.robot26.subsystems.intake.IntakeConstants.Sim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot26.subsystems.intake.IntakeConstants.Sim;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class IntakeIOSim implements IntakeIO {
  private static final DCMotor intakeGearbox = DCMotor.getKrakenX44(2);
  private final SimulatedMotorController.GenericMotorController intakeMotorController;
  private final MapleMotorSim intakeMotor;
  private Voltage intakeAppliedVoltage = Volts.of(0);

  private static final DCMotor deployGearbox = DCMotor.getKrakenX44(1);
  private final SimulatedMotorController.GenericMotorController deployMotorController;
  private final MapleMotorSim deployMotor;
  private Voltage deployAppliedVoltage = Volts.of(0);

  private final IntakeSimulation intakeSimulation;

  // one is the actual simulator and one is like the which model is used and its
  // gearbox configuration, using both flywheelsim and maple motor sim is good
  private final FlywheelSim intakeSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(intakeGearbox, 0.1, GEARING_INTAKE),
          intakeGearbox,
          0.000015);

  private final FlywheelSim deploySim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(deployGearbox, 0.1, GEARING_DEPLOY),
          deployGearbox,
          0.000015);

  public IntakeIOSim(AbstractDriveTrainSimulation driveTrain) {
    // Here, create the intake simulation with respect to the intake on your real robot
    this.intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            // Specify the type of game pieces that the intake can collect
            "Fuel",
            // Specify the drivetrain to which this intake is attached
            driveTrain,
            // Width of the intake
            Meters.of(0.7),
            // The extension length of the intake beyond the robot's frame (when activated)
            Meters.of(0.2),
            // The intake is mounted on the back side of the chassis
            IntakeSimulation.IntakeSide.BACK,
            // The intake can hold up to 1 note
            1);

    intakeMotor =
        new MapleMotorSim(
            new SimMotorConfigs(
                intakeGearbox, GEARING_INTAKE, Sim.MOTOR_LOAD_MOI, Sim.FRICTION_VOLTAGE));
    intakeMotorController =
        intakeMotor.useSimpleDCMotorController().withCurrentLimit(SUPPLY_CURRENT_LIMIT);

    deployMotor =
        new MapleMotorSim(
            new SimMotorConfigs(
                deployGearbox, GEARING_DEPLOY, Sim.MOTOR_LOAD_MOI, Sim.FRICTION_VOLTAGE));
    deployMotorController =
        deployMotor.useSimpleDCMotorController().withCurrentLimit(SUPPLY_CURRENT_LIMIT);
  }

  boolean isDeployed = false;

  @Override
  public void setIntakeOpenLoop(Voltage output) {
    intakeAppliedVoltage = output;
    if (isDeployed) {
      intakeSimulation.setIntakeVoltage(
          output); // Set the intake voltage based on the applied voltage
    } else {
      intakeSimulation.setIntakeVoltage(Volts.of(0));
    } // If the intake is not deployed, set voltage to 0
  }

  @Override
  public void setIntakeClosedLoop(AngularVelocity velocity) {
    intakeAppliedVoltage = Volts.of(velocity.in(RPM) * 0.01); // Convert RPM to voltage (simplified)
    setIntakeOpenLoop(intakeAppliedVoltage); // Use the open loop method to set the voltage
  }

  @Override
  public void setDeployOpenLoop(Voltage output) {
    deployAppliedVoltage = output;
    if (output.in(Volts) > 0.5) {
      isDeployed = true;
    } else if (output.in(Volts) < -0.5) {
      isDeployed = false;
    }
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeMotorController.requestVoltage(intakeAppliedVoltage);
    intakeSim.setInputVoltage(intakeMotor.getAppliedVoltage().in(Volts));
    intakeMotor.update(Seconds.of(TimedRobot.kDefaultPeriod));
    intakeSim.update(TimedRobot.kDefaultPeriod);

    deployMotorController.requestVoltage(deployAppliedVoltage);
    deploySim.setInputVoltage(deployMotor.getAppliedVoltage().in(Volts));
    deployMotor.update(Seconds.of(TimedRobot.kDefaultPeriod));
    deploySim.update(TimedRobot.kDefaultPeriod);

    var intakeAngularVelocity = intakeSim.getAngularVelocityRadPerSec();
    var deployAngularVelocity = deploySim.getAngularVelocityRadPerSec();

    // Update motor inputs
    inputs.intakeConnected = true;
    inputs.intakeAppliedVolts = intakeAppliedVoltage;
    inputs.intakeCurrent = Amps.of(intakeSim.getCurrentDrawAmps());
    inputs.intakeVelocity = AngularVelocity.ofBaseUnits(intakeAngularVelocity, RadiansPerSecond);

    inputs.deployConnected = true;
    inputs.deployAppliedVolts = deployAppliedVoltage;
    inputs.deployCurrent = Amps.of(deploySim.getCurrentDrawAmps());
    inputs.deployVelocity = AngularVelocity.ofBaseUnits(deployAngularVelocity, RadiansPerSecond);
    // TODO: set deploy position
  }
}
