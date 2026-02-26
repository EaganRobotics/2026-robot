package frc.robot26.subsystems.floor;

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

  public Command setClosedLoop(AngularVelocity velocity) {
    return this.startEnd(
            () -> {
              io.setFloorClosedLoop(velocity);
            },
            () -> {
              io.setFloorClosedLoop(RPM.of(0));
            })
        .withName("Floor.setClosedLoop");
  }

  public Command setTunableFloor() {
    return Commands.defer(
        () -> this.setClosedLoop(RPM.of(FloorConstants.floorSpeedRPM.get())), Set.of(this));
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
