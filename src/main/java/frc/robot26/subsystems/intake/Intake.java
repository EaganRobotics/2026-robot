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
  private DeployState currentState = DeployState.RETRACTED;

  public static enum DeployState {
    EXTENDED,
    RETRACTED
  }

  public Intake(IntakeIO io) {
    this.io = io;

    limitHit.onTrue(
        Commands.runOnce(
                () -> {
                  System.out.println(
                      "[Intake] Limit hit, setting state to RETRACTED and setting motor volts to 0");
                  currentState = DeployState.RETRACTED;
                  io.setDeployOpenLoop(Volts.of(0));
                  // io.zeroEncoder();

                })
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

  public Command setDeployPosition(DeployState state) {
    return this.runOnce(
            () -> {
              currentState = state;
              io.setDeployPosition(state);
            })
        .andThen(state == DeployState.RETRACTED ? Commands.waitUntil(limitHit) : Commands.none())
        .andThen(
            Commands.runOnce(
                () -> {
                  io.setDeployOpenLoop(Volts.of(0));
                }))
        .withName("Intake.setDeployPosition");
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
