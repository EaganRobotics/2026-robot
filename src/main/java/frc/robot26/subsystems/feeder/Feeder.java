package frc.robot26.subsystems.feeder;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  public Feeder(FeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  public Command setOpenLoop(Voltage output) {
    return this.startEnd(
            () -> {
              io.setFeederOpenLoop(output);
            },
            () -> {
              io.setFeederOpenLoop(Volts.of(0));
            })
        .withName("Feeder.setOpenLoop");
  }

  public Command setJoystickOpenLoop(DoubleSupplier speed) {
    return this.runEnd(
            () -> {
              io.setFeederOpenLoop(
                  Volts.of(speed.getAsDouble() * 12.0 * FeederConstants.joystickSpeedMultiplier));
            },
            () -> {
              io.setFeederOpenLoop(Volts.of(0));
            })
        .withName("Feeder.setJoystickOpenLoop");
  }
}
