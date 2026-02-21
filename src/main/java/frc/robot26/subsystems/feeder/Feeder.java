package frc.robot26.subsystems.feeder;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  private AngularVelocity velocitySetpoint = RPM.of(0);

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

  public Command setClosedLoop(AngularVelocity velocity) {
    return this.startEnd(
            () -> {
              io.setFeederClosedLoop(velocity);
            },
            () -> {
              io.setFeederClosedLoop(RPM.of(0));
            })
        .withName("Feeder.setClosedLoop");
  }

  public Command setTunableFeeder() {
    return Commands.defer(
        () -> this.setClosedLoop(RPM.of(FeederConstants.Real.feederSpeed.get())), Set.of(this));
  }

  public Trigger isAtVelocitySetpoint() {
    return new Trigger(
        () -> {
          return inputs.feederVelocity.gte(velocitySetpoint);
        });
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
