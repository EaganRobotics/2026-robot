package frc.lib.devices;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DigitalInputWrapper {
  int channel;
  private boolean inverted;
  DigitalInput dInput;

  public DigitalInputWrapper(int channel, String name, boolean inverted) {
    this.channel = channel;
    this.inverted = inverted;
    dInput = new DigitalInput(channel);
    Shuffleboard.getTab("sensors").addBoolean(name, this::get);
  }

  public boolean get() {
    if (inverted) {
      return !dInput.get();
    } else {
      return dInput.get();
    }
  }

  public Trigger asTrigger() {
    return new Trigger(this::get);
  }

  // GenericEntry digInputStatus = Shuffleboard.getTab("sensors").addBoolean(name,
  // dInput.get());
}
