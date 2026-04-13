package frc.robot26.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public boolean feederConnected = false;
    public AngularVelocity feederVelocity = RadiansPerSecond.of(0.0);
    public Voltage feederAppliedVolts = Volts.of(0.0);
    public Current feederCurrent = Amps.of(0.0);
    public boolean compliantWheelConnected = false;
    public AngularVelocity compliantWheelVelocity = RadiansPerSecond.of(0.0);
    public Voltage compliantWheelAppliedVolts = Volts.of(0.0);
    public Current compliantWheelCurrent = Amps.of(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FeederIOInputs inputs) {}

  public default void setFeederOpenLoop(Voltage output) {}

  public default void setFeederClosedLoop(AngularVelocity velocity) {}
}
