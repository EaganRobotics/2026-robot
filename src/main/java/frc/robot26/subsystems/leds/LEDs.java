package frc.robot26.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class LEDs extends SubsystemBase {
  private final LEDsIO io;
  private final LEDsIOInputsAutoLogged inputs = new LEDsIOInputsAutoLogged();

  public LEDs(LEDsIO io) {
    this.io = io;
  }

  public Command setColor(Supplier<Color> colorSupplier) {
    return this.runEnd(
            () -> {
              Color color = colorSupplier.get();
              io.setColor(
                  new Color8Bit(
                      (int) (color.red * 255),
                      (int) (color.green * 255),
                      (int) (color.blue * 255)));
            },
            () -> {
              io.setColor(new Color8Bit());
            })
        .ignoringDisable(true)
        .withName("LEDs.setColor");
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("LEDs", inputs);
  }
}
