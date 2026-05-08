package frc.robot26.commands;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot26.subsystems.feeder.FeederConstants.Real.feederSpeed;
import static frc.robot26.subsystems.shooter.ShooterConstants.Real.hoodAngle;
import static frc.robot26.subsystems.shooter.ShooterConstants.Real.shooterPreSpinAcceleration;
import static frc.robot26.subsystems.shooter.ShooterConstants.Real.shooterPreSpinSpeed;
import static frc.robot26.subsystems.shooter.ShooterConstants.Real.shooterSpeed;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot26.subsystems.drive.Drive;
import frc.robot26.subsystems.feeder.Feeder;
import frc.robot26.subsystems.floor.Floor;
import frc.robot26.subsystems.shooter.Shooter;
import frc.robot26.subsystems.shooter.ShooterConstants;
import frc.robot26.subsystems.shooter.setpoint.ShooterDistanceTable;
import frc.robot26.subsystems.shooter.setpoint.ShooterSetpoint;
import frc.robot26.subsystems.vision.VisionConstants;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public final class ShooterCommands {
  private static final double DEADBAND = 0.1;

  public static Command shootAutoAim(Shooter shooter, Floor floor, Feeder feeder, Drive drive) {
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
                          Commands.waitSeconds(1.5)
                              .andThen(feeder.setClosedLoop(setpoint.feederSpeed)))
                      .alongWith(floor.setClosedLoop(RPM.of(5000))));
        },
        Set.of(shooter, feeder, floor));
  }

  public static Command shootAutoAimContinuous(
      Shooter shooter, Floor floor, Feeder feeder, Drive drive) {
    return Commands.defer(
        () -> {
          ShooterSetpoint setpoint = getSetpoint(drive);
          return shooter
              .setShooterClosedLoopAndAngle(() -> setpoint.shooterSpeed, () -> setpoint.hoodAngle)
              .alongWith(
                  Commands.waitUntil(
                          shooter.isAtVelocitySetpoint(setpoint.shooterSpeed, 0.9)::getAsBoolean)
                      .andThen(feeder.setClosedLoop(() -> setpoint.feederSpeed)))
              .alongWith(floor.setClosedLoop(RPM.of(6000)));
        },
        Set.of(shooter, feeder, floor));
  }

  public static Command shootAutoAimVollyContinuous(
      Shooter shooter, Floor floor, Feeder feeder, Drive drive) {
    return Commands.defer(
        () -> {
          ShooterSetpoint setpoint = getSetpoint(drive);
          AngularVelocity vollySpeed = setpoint.shooterSpeed.plus(RPM.of(100));
          return shooter
              .setShooterClosedLoopAndAngle(() -> vollySpeed, () -> setpoint.hoodAngle)
              .alongWith(
                  Commands.waitUntil(shooter.isAtVelocitySetpoint(vollySpeed, 0.9)::getAsBoolean)
                      .andThen(feeder.setClosedLoop(() -> setpoint.feederSpeed)))
              .alongWith(floor.setClosedLoop(RPM.of(6000)));
        },
        Set.of(shooter, feeder, floor));
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

  public static Command shooterAllianceSideDefaultCommand(
      Shooter shooter, Drive drive, DoubleSupplier manualSupplier) {
    return shooter.runEnd(
        () -> {
          double manualInput = MathUtil.applyDeadband(manualSupplier.getAsDouble(), DEADBAND);
          boolean canPreSpin = DriverStation.isTeleopEnabled() && isOnAllianceSide(drive);
          boolean isManualOverride = Math.abs(manualInput) > 0.0;
          boolean shouldPreSpin = canPreSpin && !isManualOverride;

          Logger.recordOutput("Shooter/AllianceSidePreSpinEnabled", shouldPreSpin);
          Logger.recordOutput("Shooter/AllianceSideRobotX", drive.getPose().getX());

          if (isManualOverride) {
            shooter.applyShooterOpenLoop(
                Volts.of(manualInput * 12.0 * ShooterConstants.joystickSpeedMultiplier));
            Logger.recordOutput("Shooter/AllianceSidePreSpinTargetRPM", 0.0);
            return;
          }

          if (shouldPreSpin) {
            shooter.applyShooterClosedLoop(
                RPM.of(shooterPreSpinSpeed.get()), shooterPreSpinAcceleration.get());
            Logger.recordOutput("Shooter/AllianceSidePreSpinTargetRPM", shooterPreSpinSpeed.get());
            return;
          }

          shooter.stopShooter();
          Logger.recordOutput("Shooter/AllianceSidePreSpinTargetRPM", 0.0);
        },
        () -> {
          shooter.stopShooter();
          Logger.recordOutput("Shooter/AllianceSidePreSpinEnabled", false);
          Logger.recordOutput("Shooter/AllianceSidePreSpinTargetRPM", 0.0);
        });
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

  private static boolean isOnAllianceSide(Drive drive) {
    double midlineX = VisionConstants.aprilTagLayout.getFieldLength() / 2.0;
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    double robotX = drive.getPose().getX();
    return alliance == Alliance.Red ? robotX >= midlineX : robotX <= midlineX;
  }
}
