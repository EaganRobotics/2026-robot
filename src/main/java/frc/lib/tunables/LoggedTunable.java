package frc.lib.tunables;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import java.util.Iterator;
import java.util.List;
import java.util.function.Consumer;

interface LoggedTunable<T> {

  List<Consumer<T>> getListeners();

  T getValue();

  T getDefaultValue();

  public default void addListener(Consumer<T> listener) {
    getListeners().add(listener);

    // Only do tunables when not in a match, when in a match, give listener the default value
    if (DriverStation.getMatchType() == MatchType.None) {
      listener.accept(getValue());
    } else {
      listener.accept(getDefaultValue());
    }
  }

  public default void removeListener(Consumer<T> listener) {
    Iterator<Consumer<T>> iter = getListeners().iterator();
    while (iter.hasNext()) {
      Consumer<T> current = iter.next();
      if (current == null || current == listener) {
        iter.remove();
      }
    }
  }

  default void notifyListeners() {
    Iterator<Consumer<T>> iter = getListeners().iterator();
    while (iter.hasNext()) {
      Consumer<T> listener = iter.next();
      if (listener != null) {
        listener.accept(getValue());
      } else {
        iter.remove();
      }
    }
  }
}
