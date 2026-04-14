package frc.robot26.subsystems.leds;

import static edu.wpi.first.units.Units.Amps;

public class LEDsIOSim implements LEDsIO {
  public LEDsIOSim() {}

  @Override
  public void updateInputs(LEDsIOInputs inputs) {
    inputs.ledsConnected = true;
    inputs.ledsCurrent = Amps.of(1);
  }
}
