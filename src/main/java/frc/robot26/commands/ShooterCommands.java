package frc.robot26.commands;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.RPM;
import static frc.robot26.subsystems.feeder.FeederConstants.Real.feederSpeed;
import static frc.robot26.subsystems.shooter.ShooterConstants.Real.hoodAngle;
import static frc.robot26.subsystems.shooter.ShooterConstants.Real.shooterSpeed;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot26.subsystems.drive.Drive;
import frc.robot26.subsystems.feeder.Feeder;
import frc.robot26.subsystems.floor.Floor;
import frc.robot26.subsystems.intake.Intake;
import frc.robot26.subsystems.shooter.Shooter;
import frc.robot26.subsystems.shooter.setpoint.ShooterDistanceTable;
import frc.robot26.subsystems.shooter.setpoint.ShooterSetpoint;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public final class ShooterCommands {
  private static final double DEADBAND = 0.1;

  public static Command shootAutoAim(
      Shooter shooter, Intake intake, Floor floor, Feeder feeder, Drive drive) {
    return Commands.defer(
        () -> {
          Distance distance = SnapCommands.distanceToHub(drive);
          ShooterSetpoint setpoint = ShooterDistanceTable.getShooterSetpoint(distance);

          return shooter
              .setHoodPosition(setpoint.hoodAngle)
              .andThen(
                  shooter
                      .setShooterClosedLoop(setpoint.shooterSpeed)
                      .alongWith(
                          Commands.waitSeconds(0.6)
                              .andThen(feeder.setClosedLoop(setpoint.feederSpeed)))
                      .alongWith(floor.setClosedLoop(RPM.of(6000)))
                      .alongWith(intake.setIntakeClosedLoop(RPM.of(6000))));
        },
        Set.of(shooter, intake, feeder, floor));
  }

  public static Command shootAutoAimContinuous(
      Shooter shooter, Intake intake, Floor floor, Feeder feeder, Drive drive) {
    return Commands.defer(
        () -> {
          return shooter
              .setHoodPosition(() -> getSetpoint(drive).hoodAngle)
              .andThen(
                  shooter
                      .setShooterClosedLoop(() -> getSetpoint(drive).shooterSpeed)
                      .alongWith(
                          Commands.waitSeconds(0.6)
                              .andThen(feeder.setClosedLoop(() -> getSetpoint(drive).feederSpeed)))
                      .alongWith(floor.setClosedLoop(RPM.of(6000)))
                      .alongWith(intake.setIntakeClosedLoop(RPM.of(6000))));
        },
        Set.of(shooter, intake, feeder, floor));
  }

  private static ShooterSetpoint getSetpoint(Drive drive) {
    Distance distance = SnapCommands.distanceToHub(drive);
    ShooterSetpoint setpoint = ShooterDistanceTable.getShooterSetpoint(distance);
    return setpoint;
  }

  public static Command shootManualAim(Shooter shooter, Floor floor, Feeder feeder, Drive drive) {

    return shooter
        .setShooterClosedLoop(RPM.of(shooterSpeed.get()))
        .andThen(
            feeder
                .setClosedLoop(RPM.of(feederSpeed.get()))
                .andThen(shooter.setHoodPosition(Degree.of(hoodAngle.get()))));
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
            return RPM.of(shooterSpeed.get() * linearMagnitude);
          }

          if (distanceSupplier.get().lte(radius)) {
            return RPM.of(shooterSpeed.get() * 0.75);
          }

          return RPM.of(0);
        });
  }
}
