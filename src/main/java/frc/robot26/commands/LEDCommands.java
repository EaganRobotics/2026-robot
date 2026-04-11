package frc.robot26.commands;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot26.subsystems.leds.LEDs;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.function.BooleanSupplier;

public class LEDCommands {
  // Blue: normally
  // Green: limelight detecting something
  // Orange: something not plugged in
  // Red: browning out
  public static Command defaultCommand(LEDs leds, BooleanSupplier hasSeenAprilTag) {
    return leds.setColor(
        () -> {
          if (RobotController.isBrownedOut()) {
            return Color.kRed;
          }

          if (!isUSBPluggedIn()) {
            return Color.kOrange;
          }

          if (hasSeenAprilTag.getAsBoolean()) {
            return Color.kGreen;
          }

          return new Color(0.0, 35.0 / 255, 105.0 / 255); // blue
        });
  }

  public static boolean isUSBPluggedIn() {
    if (RobotBase.isSimulation()) {
      return true;
    } else {
      try {
        // prefer a mounted USB drive if one is accessible
        Path usbDir = Paths.get("/u").toRealPath();
        if (Files.isWritable(usbDir)) {
          return true;
        } else {
          return false;
        }
      } catch (IOException ex) {
        return false;
      }
    }
  }
}
