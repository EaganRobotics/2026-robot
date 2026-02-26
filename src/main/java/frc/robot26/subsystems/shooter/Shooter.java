package frc.robot26.subsystems.shooter;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private AngularVelocity velocitySetpoint = RPM.of(0);

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

  public Command setShooterClosedLoop(AngularVelocity velocity) {
    return this.startEnd(
            () -> {
              io.setShooterClosedLoop(velocity);
              velocitySetpoint = velocity;
            },
            () -> {
              io.setShooterClosedLoop(RPM.of(0));
              velocitySetpoint = RPM.of(0);
            })
        .withName("Shooter.setShooterClosedLoop");
  }

  public Command setShooterClosedLoop(Supplier<AngularVelocity> velocity) {
    return this.runEnd(
            () -> {
              io.setShooterClosedLoop(velocity.get());
              velocitySetpoint = velocity.get();
            },
            () -> {
              io.setShooterClosedLoop(RPM.of(0));
              velocitySetpoint = RPM.of(0);
            })
        .withName("Shooter.setShooterClosedLoop");
  }

  public Command setTunableShooter() {
    return Commands.defer(
        () -> this.setShooterClosedLoop(RPM.of(ShooterConstants.Real.shooterSpeed.get())),
        Set.of(this));
  }

  public Trigger isAtVelocitySetpoint() {
    return new Trigger(
        () -> {
          return inputs.shooterVelocity.gte(velocitySetpoint);
        });
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
