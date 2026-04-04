package frc.robot26.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControllerCommands {
  public static Command rumble(CommandXboxController controller) {
    return Commands.startEnd(
        () -> {
          controller.setRumble(RumbleType.kBothRumble, 0.8);
        },
        () -> {
          controller.setRumble(RumbleType.kBothRumble, 0);
        });
  }
}
