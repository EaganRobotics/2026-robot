package frc.lib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class DriverStationTriggers {

  private DriverStationTriggers() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  public static Trigger isDisabled() {
    return new Trigger(DriverStation::isDisabled);
  }
}
