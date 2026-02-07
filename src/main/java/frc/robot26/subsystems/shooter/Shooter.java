package frc.robot26.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public Command setShooterOpenLoop(Voltage output) {
    return this.startEnd(
            () -> {
              io.setShooterOpenLoop(output);
            },
            () -> {
              io.setShooterOpenLoop(Volts.of(0));
            })
        .withName("Shooter.setOpenLoop");
  }

  public Command setHoodPosition(Angle angle) {
    return this.runOnce(
            () -> {
              io.setHoodPosition(angle);
            })
        .withName("Shooter.setHoodPosition");
  }

  public Command setShooterJoystickOpenLoop(DoubleSupplier speed) {
    return this.runEnd(
            () -> {
              io.setShooterOpenLoop(
                  Volts.of(speed.getAsDouble() * 12.0 * ShooterConstants.joystickSpeedMultiplier));
            },
            () -> {
              io.setShooterOpenLoop(Volts.of(0));
            })
        .withName("Shooter.setShooterJoystickOpenLoop");
  }

  public Command setHoodJoystickOpenLoop(DoubleSupplier speed) {
    return this.runEnd(
            () -> {
              io.setHoodOpenLoop(
                  Volts.of(speed.getAsDouble() * 12.0 * ShooterConstants.joystickSpeedMultiplier));
            },
            () -> {
              io.setHoodOpenLoop(Volts.of(0));
            })
        .withName("Shooter.setHoodJoystickOpenLoop");
  }
}
