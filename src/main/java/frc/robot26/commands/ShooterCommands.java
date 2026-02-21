package frc.robot26.commands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot26.subsystems.drive.Drive;
import frc.robot26.subsystems.feeder.Feeder;
import frc.robot26.subsystems.floor.Floor;
import frc.robot26.subsystems.shooter.Shooter;
import frc.robot26.subsystems.shooter.setpoint.ShooterDistanceTable;
import frc.robot26.subsystems.shooter.setpoint.ShooterSetpoint;

public class ShooterCommands {
  public static Command shootOpenLoop(Shooter shooter, Floor floor, Feeder feeder, Drive drive) {
    Distance distance = DriveCommands.distanceToHub(drive);
    ShooterSetpoint setpoint = ShooterDistanceTable.getShooterSetpoint(distance);

    return shooter
        .setShooterClosedLoop(setpoint.shooterSpeed)
        .andThen(
            feeder
                .setClosedLoop(setpoint.feederSpeed)
                .andThen(shooter.setHoodPosition(setpoint.hoodAngle)));
  }
}
