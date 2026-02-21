package frc.robot26.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean intakeConnected = false;
    public AngularVelocity intakeVelocity = RadiansPerSecond.of(0.0);
    public Voltage intakeAppliedVolts = Volts.of(0.0);
    public Current intakeCurrent = Amps.of(0.0);

    public boolean deployConnected = false;
    public AngularVelocity deployVelocity = RadiansPerSecond.of(0.0);
    public Voltage deployAppliedVolts = Volts.of(0.0);
    public Current deployCurrent = Amps.of(0.0);
    public Angle deployPosition = Rotations.of(0.0);

    public boolean limit = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setIntakeOpenLoop(Voltage output) {}

  public default void setDeployOpenLoop(Voltage output) {}

  public default void setIntakeClosedLoop(AngularVelocity velocity) {}

  public default void setDeployClosedLoop(Angle angle) {}
}
