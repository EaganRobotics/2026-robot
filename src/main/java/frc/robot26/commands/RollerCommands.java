package frc.robot26.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot26.subsystems.feeder.Feeder;
import frc.robot26.subsystems.floor.Floor;
import frc.robot26.subsystems.intake.Intake;
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
    // return shootClosedLoopDangerous(
    //     shooter, floor, feeder, shooterSetpoint, feederSetpoint, floorSetpoint);
    return shooter
        .setShooterClosedLoop(shooterSetpoint)
        .alongWith(feeder.setClosedLoop(feederSetpoint))
        .alongWith(
            Commands.waitUntil(
                    shooter
                        .isAtVelocitySetpoint(shooterSetpoint.times(0.1))
                        .and(feeder.isAtVelocitySetpoint(feederSetpoint.times(0.1))))
                .andThen(floor.setClosedLoop(floorSetpoint)))
        .withName("RollerCommands.shootClosedLoop");
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

  public static Command intakeJiggleClosedLoop(Intake intake) {
    return Commands.repeatingSequence(
        intake.setDeployClosedLoop(Inches.of(1)),
        Commands.waitSeconds(0.75),
        intake.setDeployClosedLoop(Inches.of(1)),
        Commands.waitSeconds(0.75));
  }

  public static Command intakeJiggleOpenLoop(Intake intake) {
    return Commands.repeatingSequence(
        intake.setDeployOpenLoop(Volts.of(-6)).withTimeout(0.5),
        // Commands.waitSeconds(0.1),
        intake.setDeployOpenLoop(Volts.of(6)).withTimeout(0.5),
        Commands.waitSeconds(0.1));
    // i made volts 3 not 2 lol
  }
}
