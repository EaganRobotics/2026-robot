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

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.tunables.LoggedTunableNumber;
import frc.robot26.subsystems.drive.Drive;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class SnapCommands {
  private static final Distance BLUE_HUB_CENTER_X = Inches.of(182);
  private static final Distance BLUE_HUB_CENTER_Y = Inches.of(159.0935);

  private static final Distance RED_HUB_CENTER_X = Inches.of(469);
  private static final Distance RED_HUB_CENTER_Y = Inches.of(159.0935);

  public static final LoggedTunableNumber TUNEABLE_SNAP_DISTANCE =
      new LoggedTunableNumber("Tuning/TUNEABLE_SNAP_DISTANCE_FEET", 5);

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

  private SnapCommands() {}

  private static Pose2d getRadiusTargetPose(
      Translation2d hubCenter, Translation2d robotPos, double radiusMeters) {
    Translation2d hubToRobot = robotPos.minus(hubCenter);
    double angleToRobot = Math.atan2(hubToRobot.getY(), hubToRobot.getX());

    Translation2d targetPosition =
        hubCenter.plus(
            new Translation2d(
                radiusMeters * Math.cos(angleToRobot), radiusMeters * Math.sin(angleToRobot)));

    // Heading: face the hub from the target position on the circle.
    // Both targetPosition and hubCenter are fixed at defer time so this angle is
    // perfectly stable — no per-cycle noise.
    double angleToHub =
        Math.atan2(
            hubCenter.getY() - targetPosition.getY(), hubCenter.getX() - targetPosition.getX());

    return new Pose2d(targetPosition, new Rotation2d(angleToHub));
  }

  public static Command snapToRadius(Drive drive, Distance radius) {
    return Commands.defer(
            () -> {
              Translation2d hubCenter = getHubCenter();
              double radiusMeters = radius.in(Meters);
              Translation2d robotPos = drive.getPose().getTranslation();

              Pose2d innerPose = getRadiusTargetPose(hubCenter, robotPos, radiusMeters);

              Logger.recordOutput("SnapToRadius/InnerPose", innerPose);
              Logger.recordOutput("SnapToRadius/DesiredRadius", radiusMeters);

              return SnapToPositionTemplate.snapToPosition(drive, innerPose);
            },
            Set.of(drive))
        .withName("DriveCommands.snapToRadius");
  }

  public static Command tuneableSnapToRadius(Drive drive) {
    return Commands.defer(
            () -> {
              Translation2d hubCenter = getHubCenter();
              double radiusMeters = Feet.of(TUNEABLE_SNAP_DISTANCE.get()).in(Meters);
              Translation2d robotPos = drive.getPose().getTranslation();

              Pose2d innerPose = getRadiusTargetPose(hubCenter, robotPos, radiusMeters);

              Logger.recordOutput("SnapToRadius/InnerPose", innerPose);
              Logger.recordOutput("SnapToRadius/DesiredRadius", radiusMeters);

              return SnapToPositionTemplate.snapToPosition(drive, innerPose);
            },
            Set.of(drive))
        .withName("DriveCommands.snapToRadius");
  }

  public static Command snapToRadiusInterpolation(Drive drive, Distance radius) {
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

              Pose2d outerPose =
                  new Pose2d(
                      targetPosition.plus(new Translation2d(-1, -1)), new Rotation2d(angleToHub));
              Pose2d innerPose = new Pose2d(targetPosition, new Rotation2d(angleToHub));

              double interpolateTime =
                  robotPos.getDistance(innerPose.getTranslation()) > 1.5 ? 0.75 : 0.35;

              Logger.recordOutput("SnapToRadius/OuterPose", outerPose);
              Logger.recordOutput("SnapToRadius/InnerPose", innerPose);
              Logger.recordOutput("SnapToRadius/DesiredRadius", radiusMeters);

              return SnapToPositionTemplate.snapToPositionInterpolation(
                  drive, outerPose, innerPose, interpolateTime);
            },
            Set.of(drive))
        .withName("DriveCommands.snapToRadiusInterpolation");
  }

  private static Translation2d getHubCenter() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
        ? RED_HUB_CENTER
        : BLUE_HUB_CENTER;
  }
}
