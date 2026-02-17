package frc.robot26.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean shooterConnected = false;
    public AngularVelocity shooterVelocity = RadiansPerSecond.of(0.0);
    public Voltage shooterAppliedVolts = Volts.of(0.0);
    public Current shooterCurrent = Amps.of(0.0);

    public boolean hoodConnected = false;
    public AngularVelocity hoodVelocity = RadiansPerSecond.of(0.0);
    public Voltage hoodAppliedVolts = Volts.of(0.0);
    public Current hoodCurrent = Amps.of(0.0);
    public Angle hoodPosition = Rotations.of(0.0);
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterOpenLoop(Voltage output) {}

  public default void setHoodOpenLoop(Voltage output) {}

  public default void setHoodPosition(Angle angle) {}

  public default void setShooterClosedLoop(AngularVelocity velocity) {}
}
