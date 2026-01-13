package frc.lib.tunables;

import java.lang.ref.WeakReference;
import java.util.Iterator;
import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;

interface LoggedTunable<T> {

    List<WeakReference<Consumer<T>>> getListeners();

    T getValue();

    T getDefaultValue();

    public default void addListener(Consumer<T> listener) {
        getListeners().add(new WeakReference<>(listener));

        // Only do tunables when not in a match, when in a match, give listener the
        // default value
        if (DriverStation.getMatchType() == MatchType.None) {
            listener.accept(getValue());
        } else {
            listener.accept(getDefaultValue());
        }
    }

    public default void removeListener(Consumer<T> listener) {
        Iterator<WeakReference<Consumer<T>>> iter = getListeners().iterator();
        while (iter.hasNext()) {
            WeakReference<Consumer<T>> ref = iter.next();
            Consumer<T> current = ref.get();
            if (current == null || current == listener) {
                iter.remove();
            }
        }
    }

    default void notifyListeners() {
        Iterator<WeakReference<Consumer<T>>> iter = getListeners().iterator();
        while (iter.hasNext()) {
            WeakReference<Consumer<T>> ref = iter.next();
            Consumer<T> listener = ref.get();
            if (listener != null) {
                listener.accept(getValue());
            } else {
                iter.remove();
            }
        }
    }
}
