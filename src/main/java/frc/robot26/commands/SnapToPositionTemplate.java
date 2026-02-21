// Copyright 2021-2026 FRC 2220
// http://github.com/EaganRobotics
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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.tunables.LoggedTunablePIDs;
import frc.robot26.subsystems.drive.Drive;
import java.util.Optional;
import java.util.Set;
import org.littletonrobotics.junction.Logger;

public class SnapToPositionTemplate {
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double ANGLE_TOLERANCE = Degrees.of(3).in(Radians);
  private static final double POSITION_MAX_VELOCITY = 4.5;
  private static final double POSITION_MAX_ACCELERATION = 6;
  private static final double POSITION_TOLERANCE = Inches.of(1).in(Meters);

  // PID controllers
  public static final double X_KP = 0.5;
  public static final double X_KI = 0.0;
  public static final double X_KD = 0.0;

  public static final double Y_KP = 0.5;
  public static final double Y_KI = 0.0;
  public static final double Y_KD = 0.0;

  public static final double ANGLE_KP = 0.5;
  public static final double ANGLE_KI = 0.0;
  public static final double ANGLE_KD = 0.4;

  private static final LoggedTunablePIDs xPIDs =
      new LoggedTunablePIDs("SnapToPosition/X", X_KP, X_KI, X_KD);
  private static final LoggedTunablePIDs yPIDs =
      new LoggedTunablePIDs("SnapToPosition/Y", Y_KP, Y_KI, Y_KD);
  private static final LoggedTunablePIDs anglePIDs =
      new LoggedTunablePIDs("SnapToPosition/Angle", ANGLE_KP, ANGLE_KI, ANGLE_KD);

  public static final ProfiledPIDController xController =
      xPIDs.createController(POSITION_MAX_VELOCITY, POSITION_MAX_ACCELERATION);
  public static final ProfiledPIDController yController =
      yPIDs.createController(POSITION_MAX_VELOCITY, POSITION_MAX_ACCELERATION);
  public static final ProfiledPIDController angleController =
      anglePIDs.createController(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION);

  static {
    // Setup PID controllers
    xController.setTolerance(POSITION_TOLERANCE);
    yController.setTolerance(POSITION_TOLERANCE);
    angleController.enableContinuousInput(-Math.PI, Math.PI);
    angleController.setTolerance(ANGLE_TOLERANCE);
  }

  // EXAMPLE OF HOW TO MAKE POSITION ARRAYS FOR SNAP TO POSITION:

  // public static Pose2d[] makeReefPositions(Distance reefOffset) {
  // Transform2d REEF_BRANCH_TO_ROBOT = new Transform2d(
  // Inches.of(-INCHES_FROM_REEF).minus(reefOffset), Inches.zero(),
  // Rotation2d.kZero);
  // return new Pose2d[] {
  // new Pose2d(
  // BLUE_REEF_CENTER.plus(
  // new Translation2d(Inches.of(-20.738000), Inches.of(6.482000))),
  // Rotation2d.kZero).transformBy(REEF_BRANCH_TO_ROBOT),
  // new Pose2d(
  // BLUE_REEF_CENTER.plus(
  // new Translation2d(Inches.of(-20.738000), Inches.of(-6.482000))),
  // Rotation2d.kZero).transformBy(REEF_BRANCH_TO_ROBOT)};
  // }

  // private static final Pose2d[] REEF_POSITIONS =
  // makeReefPositions(Inches.of(12));

  private SnapToPositionTemplate() {}

  public static final class Pose2dSequence {
    public final Pose2d inner;
    public final Pose2d outer;

    public Pose2dSequence(Pose2d inner, Pose2d outer) {
      this.inner = inner;
      this.outer = outer;
    }

    public static final Pose2dSequence kZero = new Pose2dSequence(Pose2d.kZero, Pose2d.kZero);
  }

  private static Optional<Integer> getClosestPositionIndex(
      Drive drive, Pose2d[] positions, Distance maxRadius) {
    Optional<Integer> closestIndex = Optional.empty();
    Distance minDistance = Meters.of(Double.MAX_VALUE);

    for (int i = 0; i < positions.length; i++) {
      Pose2d pose = positions[i];
      double distance = drive.getPose().getTranslation().getDistance(pose.getTranslation());
      Distance distanceMeasure = Meters.of(distance);

      if (distanceMeasure.lte(maxRadius) && distanceMeasure.lt(minDistance)) {
        minDistance = distanceMeasure;
        closestIndex = Optional.of(i);
      }
    }

    return closestIndex;
  }

  private static Optional<Pose2dSequence> getClosestPositionSequence(
      Drive drive, Pose2d[] outerPositions, Pose2d[] innerPositions, Distance maxRadius) {

    if (outerPositions.length != innerPositions.length) {
      throw new IllegalArgumentException(
          "Outer and inner position arrays must have the same length");
    }

    Optional<Integer> closestIndex = getClosestPositionIndex(drive, outerPositions, maxRadius);

    return closestIndex.map(
        index -> new Pose2dSequence(innerPositions[index], outerPositions[index]));
  }

  public static Command snapToClosestPosition(
      Drive drive, Pose2d[] outerPositions, Pose2d[] innerPositions, Distance maxRadius) {
    return Commands.defer(
            () -> {
              Pose2dSequence poses =
                  getClosestPositionSequence(drive, outerPositions, innerPositions, maxRadius)
                      .orElse(Pose2dSequence.kZero);

              Logger.recordOutput("FlySnap/OuterPose", poses.outer);
              Logger.recordOutput("FlySnap/InnerPose", poses.inner);

              double interpolateTime =
                  drive.getPose().getTranslation().getDistance(poses.outer.getTranslation()) > 1.5
                      ? 1.5
                      : 0.75;
              return snapToPosition(drive, poses.outer, poses.inner, interpolateTime);
            },
            Set.of(drive))
        .withName("snapToPositionTemplate.snapToClosestPosition");
  }

  public static Command snapToPosition(
      Drive drive, Pose2d outerPose, Pose2d innerPose, double interpolateTime) {
    return Commands.defer(
        () -> {
          double startTime = Timer.getFPGATimestamp();

          return Commands.run(
                  () -> {
                    double elapsedTime = (Timer.getFPGATimestamp() - startTime);
                    double t = Math.min(1.0, elapsedTime / interpolateTime);

                    Pose2d desiredPose = outerPose.interpolate(innerPose, t);

                    var x = xController.calculate(drive.getPose().getX(), desiredPose.getX());
                    var y = yController.calculate(drive.getPose().getY(), desiredPose.getY());
                    var omega =
                        angleController.calculate(
                            drive.getRotation().getRadians(),
                            desiredPose.getRotation().getRadians());

                    Logger.recordOutput("FlySnap/DesiredPose", desiredPose);
                    Logger.recordOutput("FlySnap/InterpolateT", t);

                    ChassisSpeeds speeds = new ChassisSpeeds(x, y, omega);
                    drive.runVelocity(
                        ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
                  },
                  drive)
              .beforeStarting(
                  () -> {
                    angleController.reset(drive.getRotation().getRadians());
                    xController.reset(drive.getPose().getX());
                    yController.reset(drive.getPose().getY());
                  })
              .until(() -> angleController.atGoal() && xController.atGoal() && yController.atGoal())
              .withName("snapToPositionTemplate.snapToPosition");
        },
        Set.of(drive));
  }
}
