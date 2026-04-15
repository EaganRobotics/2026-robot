package frc.robot26.subsystems.leds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class LEDsIOCANdle implements LEDsIO {
  private final CANdle candle = new CANdle(LEDsConstants.Real.candleID);
  private final StatusSignal<Current> candleCurrent;

  public LEDsIOCANdle() {
    candleCurrent = candle.getOutputCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50, candleCurrent);
    ParentDevice.optimizeBusUtilizationForAll(candle);
  }

  @Override
  public void setColor(Color8Bit color) {
    candle.setControl(
        new SolidColor(LEDsConstants.Real.ledStartIndex, LEDsConstants.Real.ledEndIndex)
            .withColor(new RGBWColor(color.red, color.green, color.blue)));
  }

  @Override
  public void updateInputs(LEDsIOInputs inputs) {
    var candleConnectedStatus = BaseStatusSignal.refreshAll(candleCurrent);

    inputs.ledsConnected = candleConnectedStatus.isOK();
    inputs.ledsCurrent = candleCurrent.getValue();
  }
}
