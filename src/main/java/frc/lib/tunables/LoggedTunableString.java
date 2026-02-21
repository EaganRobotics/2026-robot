package frc.lib.tunables;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class LoggedTunableString extends LoggedNetworkString implements LoggedTunable<String> {

  private final String defaultValue;
  private final StringEntry entry;
  private final List<Consumer<String>> listeners;

  public String getDefaultValue() {
    return defaultValue;
  }

  public String getValue() {
    return get();
  }

  public List<Consumer<String>> getListeners() {
    return listeners;
  }

  public LoggedTunableString(String key, String defaultValue) {
    super(key, defaultValue);
    this.defaultValue = defaultValue;
    entry =
        NetworkTableInstance.getDefault()
            .getStringTopic(key)
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
