// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot26.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot26.subsystems.drive.Drive;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
  private static final double DEADBAND = 0.05;

  private static final Distance BLUE_HUB_CENTER_X = Inches.of(182);
  private static final Distance BLUE_HUB_CENTER_Y = Inches.of(159.0935);

  private static final Distance RED_HUB_CENTER_X = Inches.of(469);
  private static final Distance RED_HUB_CENTER_Y = Inches.of(159.0935);

  private static final Translation2d BLUE_HUB_CENTER =
      new Translation2d(BLUE_HUB_CENTER_X, BLUE_HUB_CENTER_Y);
  private static final Translation2d RED_HUB_CENTER =
      new Translation2d(RED_HUB_CENTER_X, RED_HUB_CENTER_Y);

  public static Distance distanceToHub(Drive drive) {

    double rx = drive.getPose().getX();
    double ry = drive.getPose().getY();
    double hx = getHubCenter().getX();
    double hy = getHubCenter().getY();
    double mx;
    double my;
    double mp;
    mx = hx - rx;
    my = hy - ry;
    mp = Math.sqrt(Math.pow(mx, 2) + Math.pow(my, 2));
    return Meters.of(mp);
  }

  // public static Pose2d[] makeReefPositions(Distance reefOffset) {
  // Transform2d REEF_BRANCH_TO_ROBOT = new Transform2d(
  // Inches.of(-INCHES_FROM_REEF).minus(reefOffset), Inches.zero(),
  // Rotation2d.kZero);
  // return new Pose2d[] {
  // new Pose2d(
  // BLUE_REEF_CENTER.plus(new Translation2d(Inches.of(-20.738000),
  // Inches.of(6.482000))),
  // Rotation2d.kZero)
  // .transformBy(REEF_BRANCH_TO_ROBOT)
  // };
  // }

  private DriveCommands() {}

  public static Command snapToRadius(Drive drive, Distance radius) {
    return Commands.defer(
            () -> {
              Translation2d hubCenter = getHubCenter();

              double radiusMeters = radius.in(Meters);

              Pose2d currentPose = drive.getPose();
              Translation2d robotPos = currentPose.getTranslation();

              Translation2d hubToRobot = robotPos.minus(hubCenter);
              double angleToRobot = Math.atan2(hubToRobot.getY(), hubToRobot.getX());

              Translation2d targetPosition =
                  hubCenter.plus(
                      new Translation2d(
                          radiusMeters * Math.cos(angleToRobot),
                          radiusMeters * Math.sin(angleToRobot)));

              double angleToHub =
                  Math.atan2(
                      hubCenter.getY() - targetPosition.getY(),
                      hubCenter.getX() - targetPosition.getX());

              Pose2d outerPose = currentPose;
              Pose2d innerPose = new Pose2d(targetPosition, new Rotation2d(angleToHub));

              double interpolateTime = robotPos.getDistance(targetPosition) > 1.5 ? 1.5 : 0.75;

              Logger.recordOutput("SnapToRadius/OuterPose", outerPose);
              Logger.recordOutput("SnapToRadius/InnerPose", innerPose);
              Logger.recordOutput("SnapToRadius/DesiredRadius", radiusMeters);

              return SnapToPositionTemplate.snapToPosition(
                  drive, outerPose, innerPose, interpolateTime);
            },
            Set.of(drive))
        .withName("DriveCommands.snapToRadius");
  }

  private static Translation2d getHubCenter() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
        ? RED_HUB_CENTER
        : BLUE_HUB_CENTER;
  }

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    // Apply deadband
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    // Square magnitude for more precise control
    linearMagnitude = linearMagnitude * linearMagnitude;

    // Return new linear velocity
    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {

    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), -ySupplier.getAsDouble());

              // Apply rotation deadband
              double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

              // Square rotation value for more precise control
              omega = Math.copySign(omega * omega, omega);

              // final double slowModeMultiplier =
              // (slowModeSupplier.getAsBoolean() ? SLOW_MODE_MULTIPLIER : 1.0);

              // No rotation
              if (Math.abs(omega) > 1E-6) {
                Logger.recordOutput("Rotation", "joystick");
                // drive.setSnapToRotation(false);
                omega *= drive.getMaxAngularSpeedRadPerSec();
                // } else if (drive.getSnapToRotation()) {
                // omega =
                // angleController.calculate(drive.getRotation().getRadians(),
                // drive.getDesiredRotation().getRadians());
                // if (angleController.atGoal()) {
                // System.out.println("Snap to rotation complete");
                // drive.setSnapToRotation(false);
                // }
              } else {
                Logger.recordOutput("Rotation", "none");
                omega = 0.0;
              }

              final double maxSpeed = drive.getMaxLinearSpeedMetersPerSec();

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * maxSpeed * -1,
                      linearVelocity.getY() * maxSpeed * -1,
                      omega);
              drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
            },
            drive)

        // Reset PID controller command starts
        .beforeStarting(
            () -> SnapToPositionTemplate.angleController.reset(drive.getRotation().getRadians()))
        .withName("DriveCommands.joyStickDrive`"); // when
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {

    // Construct command
    return Commands.run(
            () -> {
              // Get linear velocity
              Translation2d linearVelocity =
                  getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              // Calculate angular speed
              double omega =
                  SnapToPositionTemplate.angleController.calculate(
                      drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

              // Convert to field relative speeds & send command
              ChassisSpeeds speeds =
                  new ChassisSpeeds(
                      linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                      linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                      omega);
              drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
            },
            drive)
        // Reset PID controller when command starts
        .beforeStarting(
            () -> SnapToPositionTemplate.angleController.reset(drive.getRotation().getRadians()))
        .withName("DriveCommands.joystickDriveAtAngle");
  }
}
// puting this here to rebuild
