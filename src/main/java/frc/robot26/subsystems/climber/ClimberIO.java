package frc.robot26.subsystems.climber;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public boolean ClimberConnected = false;
    public AngularVelocity ClimberVelocity = RadiansPerSecond.of(0.0);
    public Voltage ClimberAppliedVolts = Volts.of(0.0);
    public Current ClimberCurrent = Amps.of(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ClimberIOInputs inputs) {}
  ;

  public default void setClimberOpenLoop(Voltage output) {}
  ;
}
