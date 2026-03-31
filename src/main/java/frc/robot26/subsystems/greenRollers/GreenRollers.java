package frc.robot26.subsystems.greenRollers;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class GreenRollers extends SubsystemBase {
  private final GreenRollersIO io;
  private final GreenRollersIOInputsAutoLogged inputs = new GreenRollersIOInputsAutoLogged();

  public GreenRollers(GreenRollersIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("GreenRollers", inputs);
  }

  public Command setOpenLoop(Voltage output) {
    return this.startEnd(
            () -> {
              io.setGreenRollersOpenLoop(output);
            },
            () -> {
              io.setGreenRollersOpenLoop(Volts.of(0));
            })
        .withName("GreenRollers.setOpenLoop");
  }

  public Command setClosedLoop(AngularVelocity velocity) {
    return this.startEnd(
            () -> {
              io.setGreenRollersClosedLoop(velocity);
            },
            () -> {
              io.setGreenRollersClosedLoop(RPM.of(0));
            })
        .withName("GreenRollers.setClosedLoop");
  }

  public Command setTunableGreenRollers() {
    return Commands.defer(
        () -> this.setClosedLoop(RPM.of(GreenRollersConstants.greenRollersSpeed.get())),
        Set.of(this));
  }

  public Command setJoystickOpenLoop(DoubleSupplier speed) {
    return this.runEnd(
            () -> {
              io.setGreenRollersOpenLoop(
                  Volts.of(
                      speed.getAsDouble() * 12.0 * GreenRollersConstants.joystickSpeedMultiplier));
            },
            () -> {
              io.setGreenRollersOpenLoop(Volts.of(0));
            })
        .withName("GreenRollers.setJoystickOpenLoop");
  }
}
