package frc.robot26.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot26.subsystems.feeder.Feeder;
import frc.robot26.subsystems.floor.Floor;
import frc.robot26.subsystems.shooter.Shooter;

public class RollerCommands {

  public static Command shootOpenLoop(Shooter shooter, Floor floor, Feeder feeder) {
    return shooter
        .setShooterOpenLoop(Volts.of(3))
        .alongWith(floor.setOpenLoop(Volts.of(3)))
        .alongWith(feeder.setOpenLoop(Volts.of(3)))
        .withName("RollerCommands.shoot");
  }

  public static Command shootClosedLoop(
      Shooter shooter,
      Floor floor,
      Feeder feeder,
      AngularVelocity shooterSetpoint,
      AngularVelocity feederSetpoint) {
    return shooter
        .setShooterClosedLoop(shooterSetpoint)
        .alongWith(feeder.setClosedLoop(feederSetpoint))
        .alongWith(
            Commands.waitUntil(shooter.isAtVelocitySetpoint().and(feeder.isAtVelocitySetpoint()))
                .andThen(floor.setOpenLoop(Volts.of(3))))
        .withName("RollerCommands.shootClosedLoop");
  }
}
