package frc.robot26.subsystems.leds;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.AutoLog;

public interface LEDsIO {
  @AutoLog
  public static class LEDsIOInputs {
    public boolean LEDsConnected = false;
    public Current LEDsCurrent = Amps.of(0.0);

    public boolean limit = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(LEDsIOInputs inputs) {}

  public default void setColor(Color8Bit color) {}
}
