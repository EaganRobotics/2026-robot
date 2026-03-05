package frc.lib.tunables;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class LoggedTunableNumber extends LoggedNetworkNumber implements LoggedTunable<Double> {

  private final double defaultValue;
  private final DoubleEntry entry;
  private final List<Consumer<Double>> listeners;

  public Double getDefaultValue() {
    return defaultValue;
  }

  public Double getValue() {
    return get();
  }

  public void incrementBy(double value) {
    set(get() + value);
  }

  public List<Consumer<Double>> getListeners() {
    return listeners;
  }

  public LoggedTunableNumber(String key, double defaultValue) {
    super(key, defaultValue);
    this.defaultValue = defaultValue;
    entry =
        NetworkTableInstance.getDefault()
            .getDoubleTopic(key)
            .getEntry(
                defaultValue, PubSubOption.keepDuplicates(false), PubSubOption.pollStorage(1));
    listeners = new ArrayList<>();
  }

  @Override
  public void periodic() {
    super.periodic();

    // Only do tunables when not in a match
    if (DriverStation.getMatchType() == MatchType.None) {
      var changes = entry.readQueueValues();
      if (changes.length == 1) {
        notifyListeners();
      }
    }
  }
}
