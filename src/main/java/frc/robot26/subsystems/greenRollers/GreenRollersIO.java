package frc.robot26.subsystems.greenRollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface GreenRollersIO {
  @AutoLog
  public static class GreenRollersIOInputs {
    public boolean greenRollersConnected = false;
    public AngularVelocity greenRollersVelocity = RadiansPerSecond.of(0.0);
    public Voltage greenRollersAppliedVolts = Volts.of(0.0);
    public Current greenRollersCurrent = Amps.of(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(GreenRollersIOInputs inputs) {}

  public default void setGreenRollersOpenLoop(Voltage output) {}

  public default void setGreenRollersClosedLoop(AngularVelocity velocity) {}
}
