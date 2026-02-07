package frc.robot26.subsystems.floor;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Floor extends SubsystemBase {
  private final FloorIO io;
  private final FloorIOInputsAutoLogged inputs = new FloorIOInputsAutoLogged();

  public Floor(FloorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Floor", inputs);
  }

  public Command setOpenLoop(Voltage output) {
    return this.startEnd(
            () -> {
              io.setFloorOpenLoop(output);
            },
            () -> {
              io.setFloorOpenLoop(Volts.of(0));
            })
        .withName("Floor.setOpenLoop");
  }

  public Command setJoystickOpenLoop(DoubleSupplier speed) {
    return this.runEnd(
            () -> {
              io.setFloorOpenLoop(
                  Volts.of(speed.getAsDouble() * 12.0 * FloorConstants.joystickSpeedMultiplier));
            },
            () -> {
              io.setFloorOpenLoop(Volts.of(0));
            })
        .withName("Floor.setJoystickOpenLoop");
  }
}
