package frc.robot26.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.ShooterSimulation;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;
import org.littletonrobotics.junction.Logger;

public class IntakeAndShooterTestSubsystem extends SubsystemBase {

  private final IntakeSimulation intakeSim;
  private final ShooterSimulation shooterSim;

  // State tracking for toggle and diagnostics
  private boolean intakeEnabled = false;
  private boolean shooterEnabled = false;
  private double intakeAppliedVolts = 0.0;
  private double shooterAppliedVolts = 0.0;

  public IntakeAndShooterTestSubsystem(AbstractDriveTrainSimulation driveTrainSimulation) {
    // Create an Intake Simulation
    // For this test, we'll model an "Over the Bumper" style intake
    // It's attached to the chassis, so it interacts with the field game pieces
    // relative to the robot's motion.
    this.intakeSim =
        IntakeSimulation.create(
                "Fuel", // gamepiece type
                driveTrainSimulation // drivesim
                )
            .capacity(5)
            .width(Meters.of(0.6))
            .extensionLength(Meters.of(0.3))
            .side(IntakeSimulation.IntakeSide.FRONT)
            .withMotor(
                new SimMotorConfigs(
                    DCMotor.getKrakenX60(1), 3.0, KilogramSquareMeters.of(0.002), Volts.of(0.5)))
            .build();

    // Register the intake to the simulation world so it can interact with game
    // pieces
    intakeSim.register();

    // Create a Shooter Simulation
    this.shooterSim =
        ShooterSimulation.create(RebuiltFuelOnField.REBUILT_FUEL_INFO)
            .onRobot(
                driveTrainSimulation,
                () ->
                    new edu.wpi.first.math.geometry.Pose3d(
                        0.2, 0, 0.5, new edu.wpi.first.math.geometry.Rotation3d()))
            .withMotor(
                new SimMotorConfigs(
                    DCMotor.getKrakenX60(2), 1.5, KilogramSquareMeters.of(0.003), Volts.of(0.5)))
            .withCapacity(5)
            .withIntakeSource(
                intakeSim) // Connect intake to shooter: pieces flow from intake -> shooter
            .build();
  }

  /** Toggle the intake on/off. */
  public void toggleIntake() {
    intakeEnabled = !intakeEnabled;
    if (intakeEnabled) {
      intakeAppliedVolts = 8.0;
      intakeSim.setIntakeVoltage(Volts.of(intakeAppliedVolts));
    } else {
      intakeAppliedVolts = 0.0;
      intakeSim.setIntakeVoltage(Volts.of(0));
    }
  }

  /** Toggle the shooter on/off. */
  public void toggleShooter() {
    shooterEnabled = !shooterEnabled;
    if (shooterEnabled) {
      shooterAppliedVolts = 12.0;
      shooterSim.setShooterVoltage(Volts.of(shooterAppliedVolts));
    } else {
      shooterAppliedVolts = 0.0;
      shooterSim.setShooterVoltage(Volts.of(0));
    }
  }

  /**
   * Run the intake at a specific voltage.
   *
   * @param volts Voltage to apply. Positive is collecting, negative is ejecting.
   */
  public void runIntake(double volts) {
    intakeAppliedVolts = volts;
    intakeSim.setIntakeVoltage(Volts.of(volts));
    intakeEnabled = Math.abs(volts) > 0.1;
  }

  /**
   * Run the shooter at a specific voltage.
   *
   * @param volts Voltage to apply.
   */
  public void runShooter(double volts) {
    shooterAppliedVolts = volts;
    shooterSim.setShooterVoltage(Volts.of(volts));
    shooterEnabled = Math.abs(volts) > 0.1;
  }

  /** Shoots a game piece if available. */
  public void shoot() {
    shooterSim.shoot(MetersPerSecond.of(6), Degrees.of(60));
  }

  /** Command to toggle intake. */
  public Command toggleIntakeCommand() {
    return this.runOnce(this::toggleIntake).withName("ToggleIntake");
  }

  /** Command to toggle shooter. */
  public Command toggleShooterCommand() {
    return this.runOnce(this::toggleShooter).withName("ToggleShooter");
  }

  /** Command to run intake. */
  public Command runIntakeCommand(double volts) {
    return this.run(() -> runIntake(volts)).finallyDo(() -> runIntake(0)).withName("RunIntake");
  }

  /** Command to run shooter. */
  public Command runShooterCommand(double volts) {
    return this.run(() -> runShooter(volts)).finallyDo(() -> runShooter(0)).withName("RunShooter");
  }

  /** Command to shoot. */
  public Command shootCommand() {
    return this.runOnce(this::shoot).withName("Shoot");
  }

  @Override
  public void periodic() {
    // Comprehensive diagnostics logging
    Logger.recordOutput("TestSubsystem/Intake/Enabled", intakeEnabled);
    Logger.recordOutput("TestSubsystem/Intake/AppliedVolts", intakeAppliedVolts);
    Logger.recordOutput("TestSubsystem/Intake/IsRunning", intakeSim.isRunning());
    Logger.recordOutput("TestSubsystem/Intake/GamePieces", intakeSim.getGamePiecesAmount());
    Logger.recordOutput(
        "TestSubsystem/Intake/StatorCurrentAmps", intakeSim.getStatorCurrent().in(Amps));

    Logger.recordOutput("TestSubsystem/Shooter/Enabled", shooterEnabled);
    Logger.recordOutput("TestSubsystem/Shooter/AppliedVolts", shooterAppliedVolts);
    Logger.recordOutput("TestSubsystem/Shooter/GamePieces", shooterSim.getGamePiecesLoaded());
    Logger.recordOutput("TestSubsystem/Shooter/HasGamePiece", shooterSim.hasGamePiece());
    Logger.recordOutput(
        "TestSubsystem/Shooter/VelocityRPM",
        shooterSim.getVelocity().in(RotationsPerSecond) * 60.0);
    Logger.recordOutput(
        "TestSubsystem/Shooter/VelocityRadPerSec", shooterSim.getVelocity().in(RadiansPerSecond));
    Logger.recordOutput(
        "TestSubsystem/Shooter/StatorCurrentAmps", shooterSim.getStatorCurrent().in(Amps));
  }

  @Override
  public void simulationPeriodic() {
    // Logging simulation-specific state
    Logger.recordOutput("TestSubsystem/SimPeriodic/Called", true);
  }
}
