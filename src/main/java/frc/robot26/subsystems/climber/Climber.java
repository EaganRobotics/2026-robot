package frc.robot26.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  public Climber(ClimberIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }

  public Command setOpenLoop(Voltage output) {
    return this.startEnd(
            () -> {
              io.setClimberOpenLoop(output);
            },
            () -> {
              io.setClimberOpenLoop(Volts.of(0));
            })
        .withName("Climber.setOpenLoop");
  }

  public Command setJoystickOpenLoop(DoubleSupplier speed) {
    return this.runEnd(
            () -> {
              io.setClimberOpenLoop(
                  Volts.of(speed.getAsDouble() * 12.0 * ClimberConstants.joystickSpeedMultiplier));
            },
            () -> {
              io.setClimberOpenLoop(Volts.of(0));
            })
        .withName("Climber.setJoystickOpenLoop");
  }
}
