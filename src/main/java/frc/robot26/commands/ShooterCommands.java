package frc.robot26.commands;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.tunables.LoggedTunableNumber;
import frc.robot26.subsystems.drive.Drive;
import frc.robot26.subsystems.feeder.Feeder;
import frc.robot26.subsystems.floor.Floor;
import frc.robot26.subsystems.shooter.Shooter;
import frc.robot26.subsystems.shooter.setpoint.ShooterDistanceTable;
import frc.robot26.subsystems.shooter.setpoint.ShooterSetpoint;

public final class ShooterCommands {

  private static final LoggedTunableNumber shooterSpeed =
      new LoggedTunableNumber("shooterSpeed", 0);
  private static final LoggedTunableNumber feederSpeed = new LoggedTunableNumber("feederSpeed", 0);
  private static final LoggedTunableNumber hoodAngle = new LoggedTunableNumber("hoodAngle", 0);

  public static Command shootAutoAim(Shooter shooter, Floor floor, Feeder feeder, Drive drive) {
    Distance distance = DriveCommands.distanceToHub(drive);
    ShooterSetpoint setpoint = ShooterDistanceTable.getShooterSetpoint(distance);

    return shooter
        .setShooterClosedLoop(setpoint.shooterSpeed)
        .andThen(
            feeder
                .setClosedLoop(setpoint.feederSpeed)
                .andThen(shooter.setHoodPosition(setpoint.hoodAngle)));
  }

  public static Command shootManualAim(Shooter shooter, Floor floor, Feeder feeder, Drive drive) {

    return shooter
        .setShooterClosedLoop(RPM.of(shooterSpeed.get()))
        .andThen(
            feeder
                .setClosedLoop(RPM.of(feederSpeed.get()))
                .andThen(shooter.setHoodPosition(Degree.of(hoodAngle.get()))));
  }
}
