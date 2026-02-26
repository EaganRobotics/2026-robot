package frc.robot26.commands;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot26.subsystems.feeder.FeederConstants.Real.feederSpeedRPM;
import static frc.robot26.subsystems.shooter.ShooterConstants.Real.hoodAngleDegrees;
import static frc.robot26.subsystems.shooter.ShooterConstants.Real.shooterSpeedRPM;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot26.subsystems.drive.Drive;
import frc.robot26.subsystems.feeder.Feeder;
import frc.robot26.subsystems.floor.Floor;
import frc.robot26.subsystems.shooter.Shooter;
import frc.robot26.subsystems.shooter.setpoint.ShooterDistanceTable;
import frc.robot26.subsystems.shooter.setpoint.ShooterSetpoint;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public final class ShooterCommands {
  private static final double DEADBAND = 0.1;

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
        .setShooterClosedLoop(RPM.of(shooterSpeedRPM.get()))
        .andThen(
            feeder
                .setClosedLoop(RPM.of(feederSpeedRPM.get()))
                .andThen(shooter.setHoodPosition(Degree.of(hoodAngleDegrees.get()))));
  }

  public static Command shooterDefaultCommand(
      Shooter shooter,
      Supplier<Distance> distanceSupplier,
      DoubleSupplier doubleSupplier,
      Distance radius) {

    return shooter.setShooterClosedLoop(
        () -> {
          double linearMagnitude = MathUtil.applyDeadband(doubleSupplier.getAsDouble(), DEADBAND);
          if (Math.abs(linearMagnitude) > DEADBAND) {
            return RPM.of(shooterSpeedRPM.get() * linearMagnitude);
          }

          if (distanceSupplier.get().lte(radius)) {
            return RPM.of(shooterSpeedRPM.get() * 0.75);
          }

          return RPM.of(0);
        });
  }
}
