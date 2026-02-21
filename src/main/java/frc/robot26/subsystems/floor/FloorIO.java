package frc.robot26.subsystems.floor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface FloorIO {
  @AutoLog
  public static class FloorIOInputs {
    public boolean floorConnected = false;
    public AngularVelocity floorVelocity = RadiansPerSecond.of(0.0);
    public Voltage floorAppliedVolts = Volts.of(0.0);
    public Current floorCurrent = Amps.of(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(FloorIOInputs inputs) {}

  public default void setFloorOpenLoop(Voltage output) {}

  public default void setFloorClosedLoop(AngularVelocity velocity) {}
}
