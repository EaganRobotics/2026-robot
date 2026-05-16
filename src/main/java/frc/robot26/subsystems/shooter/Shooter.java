package frc.robot26.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot26.subsystems.shooter.ShooterConstants.Real.hoodAngle;
import static frc.robot26.subsystems.shooter.ShooterConstants.Real.hoodAngleBack;

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

  // private AngularVelocity velocitySetpoint = RPM.of(0);

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
              // velocitySetpoint = velocity;
            },
            () -> {
              io.setShooterOpenLoop(Volts.of(0));
              // io.setShooterClosedLoop(RPM.of(0));
              // velocitySetpoint = RPM.of(0);
            })
        .withName("Shooter.setShooterClosedLoop");
  }

  public Command setShooterClosedLoopAndAngle(AngularVelocity velocity, Angle angle) {
    return this.startEnd(
            () -> {
              io.setShooterClosedLoop(velocity);
              // velocitySetpoint = velocity;
              io.setHoodPosition(angle);
            },
            () -> {
              io.setShooterOpenLoop(Volts.of(0));
              // io.setShooterClosedLoop(RPM.of(0));
              // velocitySetpoint = RPM.of(0);
              io.setHoodPosition(Degrees.of(0));
            })
        .withName("Shooter.setShooterClosedLoop");

    // DOSENT DO ANYTHING
  }

  /**
   * Continuously update both shooter velocity and hood angle from suppliers. Use this when you want
   * the setpoints to be re-read each scheduler cycle.
   */
  public Command setShooterClosedLoopAndAngle(
      Supplier<AngularVelocity> velocity, Supplier<Angle> angle) {
    return this.runEnd(
            () -> {
              io.setShooterClosedLoop(velocity.get());
              // velocitySetpoint = velocity.get();
              io.setHoodPosition(angle.get());
            },
            () -> {
              io.setShooterOpenLoop(Volts.of(0));
              // io.setShooterClosedLoop(RPM.of(0));
              // velocitySetpoint = RPM.of(0);
              io.setHoodPosition(Degrees.of(0));
            })
        .withName("Shooter.setShooterClosedLoopAndAngle");
  }

  public Command setTunableShootWithHood() {
    return this.startEnd(
            () -> {
              io.setShooterClosedLoop(RPM.of(ShooterConstants.Real.shooterSpeed.get()));
              io.setHoodPosition(Degrees.of(ShooterConstants.Real.hoodAngle.get()));
            },
            () -> {
              io.setShooterOpenLoop(Volts.of(0));
              io.setHoodPosition(Degrees.of(0));
            })
        .withName("Shooter.setShooterClosedLoop2");
  }

  public Command setShooterClosedLoop(Supplier<AngularVelocity> velocity) {
    return this.runEnd(
            () -> {
              io.setShooterClosedLoop(velocity.get());
              // velocitySetpoint = velocity.get();
            },
            () -> {
              io.setShooterOpenLoop(Volts.of(0));
              // io.setShooterClosedLoop(RPM.of(0));
              // velocitySetpoint = RPM.of(0);
            })
        .withName("Shooter.setShooterClosedLoop");
  }

  public Command setTunableShooter() {
    return Commands.defer(
        () -> this.setShooterClosedLoop(RPM.of(ShooterConstants.Real.shooterSpeed.get())),
        Set.of(this));
  }

  public Trigger isAtVelocitySetpoint(AngularVelocity target, double percentage) {
    return new Trigger(() -> inputs.shooterVelocity.isNear(target, 0.6 - percentage)).debounce(0.1);
  }

  public Command setHoodPosition(Angle angle) {
    return this.runOnce(
            () -> {
              io.setHoodPosition(angle);
            })
        .withName("Shooter.setHoodPosition");
  }

  public Command setHoodPosition(Supplier<Angle> angle) {
    return this.run(
            () -> {
              io.setHoodPosition(angle.get());
            })
        .withName("Shooter.setHoodPosition");
  }

  public Command incrementSetHoodPosition(Angle angle) {
    return this.runOnce(
            () -> {
              Angle incrementedangle = inputs.hoodPosition.plus(angle);
              io.setHoodPosition(incrementedangle);
            })
        .withName("Shooter.incrementSetHoodPosition");
  }

  public Command setTunableHood() {
    return this.runOnce(
            () -> {
              io.setHoodPosition(Degrees.of(hoodAngle.get()));
            })
        .withName("Shooter.setHoodPosition");
  }

  public Command setTunableHoodBack() {
    return this.runOnce(
            () -> {
              io.setHoodPosition(Degrees.of(hoodAngleBack.get()));
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

  public Command setHoodOpenLoop(Voltage voltage) {
    return this.runEnd(
            () -> {
              io.setHoodOpenLoop(voltage);
            },
            () -> {
              io.setHoodOpenLoop(Volts.of(0));
            })
        .withName("Shooter.setHoodJoystickOpenLoop");
  }
}
