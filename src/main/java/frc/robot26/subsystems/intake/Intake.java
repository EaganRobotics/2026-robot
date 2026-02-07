package frc.robot26.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot26.subsystems.intake.IntakeConstants.DeployState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
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

  public Command setDeployPosition(DeployState state) {
    return this.runOnce(
            () -> {
              io.setDeployPosition(state);
            })
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
