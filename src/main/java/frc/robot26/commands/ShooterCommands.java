package frc.robot26.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot26.subsystems.drive.Drive;
import frc.robot26.subsystems.feeder.Feeder;
import frc.robot26.subsystems.floor.Floor;
import frc.robot26.subsystems.shooter.Shooter;

public class ShooterCommands {

  public static Command shootOpenLoop(
      Shooter shooter, Floor floor, Feeder feeder, DriveCommands driveCommands, Drive drive) {
    double DisToHub = driveCommands.distanceToHub(drive);
    double HoodAng = null;
    double ShooterRPM = null;
    double FeederRPM = null;

    return Commands.run(null, null);
  }
}
