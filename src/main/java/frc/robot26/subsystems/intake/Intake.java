package frc.robot26.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  public final Trigger limitHit = new Trigger(() -> inputs.limit);
  private IntakeIO.DeployState currentState = IntakeIO.DeployState.RETRACTED;

  public Intake(IntakeIO io) {
    this.io = io;

    limitHit.onTrue(
        Commands.runOnce(
                () -> {
                  currentState = IntakeIO.DeployState.RETRACTED;
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

  public Command setDeployPosition(IntakeIO.DeployState state) {
    return this.runOnce(
            () -> {
              currentState = state;
              io.setDeployPosition(state);
            })
        .andThen(
            state == IntakeIO.DeployState.RETRACTED
                ? Commands.waitUntil(limitHit)
                : Commands.none())
        .andThen(
            Commands.runOnce(
                () -> {
                  io.setDeployOpenLoop(Volts.of(0));
                }))
        .withName("Intake.setDeployPosition");
  }

  public Command setTunableIntake() {
    return Commands.defer(
        () -> this.setIntakeClosedLoop(RPM.of(IntakeConstants.Real.intakeSpeed.get())),
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
}
