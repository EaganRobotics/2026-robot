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
  private int lastIntakeBallCount = 0;

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
    this.intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            driveTrain,
            Meters.of(2.0), // wide for testing
            Meters.of(1.0), // long reach for testing
            IntakeSimulation.IntakeSide.BACK,
            5); // hold up to 5

    // Register with the arena so it participates in collision detection
    intakeSimulation.register();

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
    intakeSimulation.setIntakeVoltage(output);
    if (Math.abs(output.in(Volts)) > 0.5) {
      intakeSimulation.startIntake();
    } else {
      intakeSimulation.stopIntake();
    }
  }

  @Override
  public void setIntakeClosedLoop(AngularVelocity velocity) {
    intakeAppliedVoltage = Volts.of(velocity.in(RPM) * 0.01);
    setIntakeOpenLoop(intakeAppliedVoltage);
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

    // Track ball pickup — immediately transfer from intake sim to robot storage
    // so the intake sim can pick up another ball right away
    int currentBalls = intakeSimulation.getGamePiecesAmount();
    if (currentBalls > lastIntakeBallCount) {
      RobotGamePieceStorage.addBall();
      intakeSimulation.obtainGamePieceFromIntake(); // clear from intake sim so it can pick up more
      currentBalls = intakeSimulation.getGamePiecesAmount();
    }
    lastIntakeBallCount = currentBalls;

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
