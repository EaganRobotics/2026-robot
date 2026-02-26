package frc.robot26.commands;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot26.subsystems.feeder.FeederConstants.Real.feederSpeed;
import static frc.robot26.subsystems.shooter.ShooterConstants.Real.hoodAngle;
import static frc.robot26.subsystems.shooter.ShooterConstants.Real.shooterSpeed;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot26.subsystems.drive.Drive;
import frc.robot26.subsystems.feeder.Feeder;
import frc.robot26.subsystems.floor.Floor;
import frc.robot26.subsystems.shooter.Shooter;
import frc.robot26.subsystems.shooter.setpoint.ShooterDistanceTable;
import frc.robot26.subsystems.shooter.setpoint.ShooterSetpoint;
import java.util.function.Supplier;

public final class ShooterCommands {
  public static Command shootAutoAim(Shooter shooter, Floor floor, Feeder feeder, Drive drive) {
    Distance distance = SnapCommands.distanceToHub(drive);
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

  public static Command preaccelerateShooter(
      Shooter shooter, Supplier<Distance> distanceSupplier, Distance radius) {
    if (distanceSupplier.get().lte(radius)) {
      return shooter.setShooterClosedLoop(RPM.of(shooterSpeed.get() * 0.75));
    } else {
      return shooter.setShooterClosedLoop(RPM.of(0));
    }
  }
}
