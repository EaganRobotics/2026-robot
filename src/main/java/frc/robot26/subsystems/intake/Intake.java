package frc.robot26.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot26.subsystems.intake.IntakeConstants.PITCH_CIRCUMFERENCE;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot26.subsystems.intake.IntakeConstants.DeployState;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  public final Trigger limitHit = new Trigger(() -> inputs.limit);
  private DeployState currentState = DeployState.RETRACTED;

  public Intake(IntakeIO io) {
    this.io = io;

    limitHit.onTrue(
        Commands.runOnce(
                () -> {
                  currentState = DeployState.RETRACTED;
                  io.setDeployOpenLoop(Volts.of(0));
                  // io.zeroEncoder();

                },
                this)
            .withName("Intake.limitHit"));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    Logger.recordOutput("Intake/CurrentState", currentState.toString());
    Logger.recordOutput("Intake/LimitHit", inputs.limit);
    Logger.recordOutput(
        "Intake/DeployDistanceInches",
        IntakeConstants.deployDistanceFrom(inputs.deployPosition).in(Inches));
  }

  public Command setOpenLoop(Voltage output) {
    return this.startEnd(
            () -> {
              io.setIntakeOpenLoop(output);
            },
            () -> {
              io.setIntakeOpenLoop(Volts.of(0));
            })
        .withName("Intake.setOpenLoop");
  }

  public Command setDeployOpenLoop(Voltage output) {
    return this.startEnd(
            () -> {
              io.setDeployOpenLoop(output);
            },
            () -> {
              io.setDeployOpenLoop(Volts.of(0));
            })
        .withName("Intake.setDeployOpenLoop");
  }

  public Command setIntakeClosedLoop(AngularVelocity velocity) {
    return this.startEnd(
            () -> {
              io.setIntakeClosedLoop(velocity);
            },
            () -> {
              io.setIntakeClosedLoop(RPM.of(0));
            })
        .withName("Intake.setIntakeClosedLoop");
  }

  public Command setDeployClosedLoop(Distance distance) {
    return this.runOnce(
            () -> {
              double rotations = distance.in(Inches) / PITCH_CIRCUMFERENCE.in(Inches);
              io.setDeployClosedLoop(Rotations.of(rotations));
            })
        .withName("Intake.setDeployClosedLoop");
  }

  public Command setTunableIntake() {
    return Commands.defer(
        () -> this.setIntakeClosedLoop(RPM.of(IntakeConstants.Real.intakeSpeedRPM.get())),
        Set.of(this));
  }

  public Command setTunableDeploy() {
    return Commands.defer(
        () -> this.setDeployClosedLoop(Inches.of(IntakeConstants.Real.deployPositionInches.get())),
        Set.of(this));
  }

  public Command setJoystickOpenLoop(DoubleSupplier speed) {
    return this.runEnd(
            () -> {
              io.setIntakeOpenLoop(
                  Volts.of(speed.getAsDouble() * 12.0 * IntakeConstants.joystickSpeedMultiplier));
            },
            () -> {
              io.setIntakeOpenLoop(Volts.of(0));
            })
        .withName("Intake.setJoystickOpenLoop");
  }

  public Command setDeployPosition(IntakeConstants.DeployState state) {
    return setDeployClosedLoop(state.getState());
  }
}
