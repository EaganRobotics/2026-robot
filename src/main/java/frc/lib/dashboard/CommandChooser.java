package frc.lib.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class CommandChooser {

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public CommandChooser() {
    setDefaultOption(new InstantCommand().withName("Do nothing"));
  }

  public void addOption(Command command) {
    autoChooser.addOption(command.getName(), command);
  }

  public void setDefaultOption(Command command) {
    autoChooser.setDefaultOption(command.getName(), command);
  }

  public Command getSelected() {
    return autoChooser.getSelected();
  }

  public SendableChooser<Command> getSendableChooser() {
    return autoChooser;
  }
}
