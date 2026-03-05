package frc.robot26.commands;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot26.subsystems.feeder.Feeder;
import frc.robot26.subsystems.floor.Floor;
import frc.robot26.subsystems.shooter.Shooter;

public class RollerCommands {

  public static Command shootOpenLoop(Floor floor, Feeder feeder) {
    return floor
        .setOpenLoop(Volts.of(5))
        .alongWith(feeder.setOpenLoop(Volts.of(7)))
        .withName("RollerCommands.shoot");
  }

  public static Command shootClosedLoop(
      Shooter shooter,
      Floor floor,
      Feeder feeder,
      AngularVelocity shooterSetpoint,
      AngularVelocity feederSetpoint,
      AngularVelocity floorSetpoint) {
    return shootClosedLoopDangerous(
        shooter, floor, feeder, shooterSetpoint, feederSetpoint, floorSetpoint);
    // return shooter
    //     .setShooterClosedLoop(shooterSetpoint)
    //     .alongWith(feeder.setClosedLoop(feederSetpoint))
    //     .alongWith(
    //         Commands.waitUntil(shooter.isAtVelocitySetpoint().and(feeder.isAtVelocitySetpoint()))
    //             .andThen(floor.setClosedLoop(floorSetpoint)))
    //     .withName("RollerCommands.shootClosedLoop");
  }

  public static Command tuneableShootClosedLoop(
      Shooter shooter,
      Floor floor,
      Feeder feeder,
      AngularVelocity feederSetpoint,
      AngularVelocity floorSetpoint) {
    return shooter
        .setTunableShooter()
        .alongWith(feeder.setClosedLoop(feederSetpoint))
        .alongWith(floor.setClosedLoop(floorSetpoint))
        .withName("RollerCommands.shootClosedLoop");
  }

  public static Command shootClosedLoopDangerous(
      Shooter shooter,
      Floor floor,
      Feeder feeder,
      AngularVelocity shooterSetpoint,
      AngularVelocity feederSetpoint,
      AngularVelocity floorSetpoint) {
    return shooter
        .setShooterClosedLoop(shooterSetpoint)
        .alongWith(feeder.setClosedLoop(feederSetpoint))
        .alongWith(floor.setClosedLoop(floorSetpoint))
        .withName("RollerCommands.shootClosedLoopDangerous");
  }
}
