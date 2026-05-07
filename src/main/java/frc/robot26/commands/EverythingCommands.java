package frc.robot26.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot26.subsystems.drive.Drive;
import frc.robot26.subsystems.feeder.Feeder;
import frc.robot26.subsystems.floor.Floor;
import frc.robot26.subsystems.intake.Intake;
import frc.robot26.subsystems.shooter.Shooter;
import frc.robot26.subsystems.shooter.ShooterConstants;
import java.util.function.DoubleSupplier;

public class EverythingCommands {
  public static Command getTheBallsIntoTheHub(
      Drive drive,
      Shooter shooter,
      Floor floor,
      Intake intake,
      Feeder feeder,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier) {
    return DriveCommands.joystickDriveAtAngle(
            drive,
            xSupplier,
            ySupplier,
            () -> {
              Translation2d hubCenter = SnapCommands.getHubCenter();
              Translation2d hubToRobot = hubCenter.minus(drive.getPose().getTranslation());
              double distToHub = hubToRobot.getNorm();
              double leadscale = 1.6;

              // Time-of-flight estimate
              double tof = distToHub / ShooterConstants.BALL_SPEED_MPS;

              // Robot velocity in field-space
              ChassisSpeeds fieldSpeeds =
                  ChassisSpeeds.fromRobotRelativeSpeeds(
                      drive.getChassisSpeeds(), drive.getPose().getRotation());
              double vx = fieldSpeeds.vxMetersPerSecond;
              double vy = fieldSpeeds.vyMetersPerSecond;

              // Shift aim point opposite to robot velocity
              Translation2d virtualHub =
                  hubCenter.minus(new Translation2d(vx * tof * leadscale, vy * tof * leadscale));

              // Angle to virtual hub
              Translation2d virtualHubToRobot = virtualHub.minus(drive.getPose().getTranslation());
              double angleToRobot =
                  Math.PI + Math.atan2(virtualHubToRobot.getY(), virtualHubToRobot.getX());
              Rotation2d targetAngle = new Rotation2d(angleToRobot);

              // 0.5 degree deadzone
              Rotation2d currentAngle = drive.getPose().getRotation();
              double errorDegrees = Math.abs(targetAngle.minus(currentAngle).getDegrees());
              if (errorDegrees < 0.5) {
                return currentAngle;
              }
              return targetAngle;
            })
        .alongWith(ShooterCommands.shootAutoAimContinuous(shooter, floor, feeder, drive))
        .alongWith(Commands.waitSeconds(1.75).andThen(RollerCommands.intakeJiggleOpenLoop(intake)));
  }
}
