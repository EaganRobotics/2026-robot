package frc.lib.devices;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.AnalogInput;

public class PWMEncoder {
  private AnalogInput encoder;
  private Angle offset;
  private final int oversampling = 4; // Set to 1 to disable oversampling

  public PWMEncoder(int analogPort, Angle offset) {
    this.offset = offset;
    encoder = new AnalogInput(analogPort);
    encoder.setOversampleBits(oversampling);
  }

  public Angle getPosition() {
    // return encoder.getAverageValue() * 360 / 4096.0 / Math.pow(2, oversampling);
    return getRawUnoffsettedPosition().minus(offset);
  }

  public Angle getRawUnoffsettedPosition() {
    // return encoder.getAverageValue() * 360 / 4096.0 / Math.pow(2, oversampling);
    return Degrees.of(encoder.getValue() * 360.0 / 4096.0);
  }
}
