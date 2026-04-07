// Copyright 2021-2026 FRC 6328/2220
// http://github.com/Mechanical-Advantage/http://github.com/EaganRobotics
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

// shoutout kimi k2 (austraila chatgpt)
// TODO: how do you spell austraila the country -Griffin

package frc.robot26.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.tunables.LoggedTunableNumber;
import frc.lib.tunables.LoggedTunablePIDs;
import frc.robot26.subsystems.drive.Drive;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
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

  // ========== PID Constants ==========
  public static final double X_KP = 5.0;
  public static final double X_KI = 0.0;
  public static final double X_KD = 0.0;

  public static final double Y_KP = X_KP;
  public static final double Y_KI = X_KI;
  public static final double Y_KD = X_KD;

  public static final double ANGLE_KP = 10.0;
  public static final double ANGLE_KI = 0.0;
  public static final double ANGLE_KD = 0.2;

  private static final LoggedTunablePIDs xPIDs =
      new LoggedTunablePIDs("SnapToPosition/X", X_KP, X_KI, X_KD);
  private static final LoggedTunablePIDs yPIDs =
      new LoggedTunablePIDs("SnapToPosition/Y", Y_KP, Y_KI, Y_KD);
  private static final LoggedTunablePIDs anglePIDs =
      new LoggedTunablePIDs("SnapToPosition/Angle", ANGLE_KP, ANGLE_KI, ANGLE_KD);

  private static final LoggedTunableNumber TueableSnapToRadiusFeet =
      new LoggedTunableNumber("Tuning/SnapToPosition/TueableSnapToRadiusFeet", 5);

  private static final LoggedTunableNumber POSITION_MAX_VELOCITY =
      new LoggedTunableNumber("Tuning/SnapToPosition/POSITION_MAX_VELOCITY", 4.5);

  public static final LoggedTunableNumber ANGLE_MAX_VELOCITY =
      new LoggedTunableNumber("Tuning/SnapToPosition/ANGLE_MAX_VELOCITY", 8);

  public static final LoggedTunableNumber ANGLE_MAX_ACCELERATION =
      new LoggedTunableNumber("Tuning/SnapToPosition/ANGLE_MAX_ACCELERATION", 20);

  private static final LoggedTunableNumber ANGLE_TOLERANCE =
      new LoggedTunableNumber("Tuning/SnapToPosition/ANGLE_TOLERANCE", Degrees.of(3).in(Radians));

  private static final LoggedTunableNumber POSITION_MAX_ACCELERATION =
      new LoggedTunableNumber("Tuning/SnapToPosition/POSITION_MAX_ACCELERATION", 6);

  private static final LoggedTunableNumber POSITION_TOLERANCE =
      new LoggedTunableNumber("Tuning/SnapToPosition/POSITION_TOLERANCE", Inches.of(1).in(Meters));

  public static final ProfiledPIDController xController =
      xPIDs.createController(POSITION_MAX_VELOCITY.get(), POSITION_MAX_ACCELERATION.get());
  public static final ProfiledPIDController yController =
      yPIDs.createController(POSITION_MAX_VELOCITY.get(), POSITION_MAX_ACCELERATION.get());
  public static final ProfiledPIDController angleController =
      anglePIDs.createController(ANGLE_MAX_VELOCITY.get(), ANGLE_MAX_ACCELERATION.get());

  static {
    POSITION_TOLERANCE.addListener(value -> xController.setTolerance(value));
    POSITION_TOLERANCE.addListener(value -> yController.setTolerance(value));
    ANGLE_TOLERANCE.addListener(value -> angleController.setTolerance(value));

    ANGLE_MAX_VELOCITY.addListener(
        (velocity) ->
            angleController.setConstraints(
                new TrapezoidProfile.Constraints(
                    velocity, angleController.getConstraints().maxAcceleration)));
    POSITION_MAX_VELOCITY.addListener(
        (velocity) ->
            xController.setConstraints(
                new TrapezoidProfile.Constraints(
                    velocity, xController.getConstraints().maxAcceleration)));
    POSITION_MAX_VELOCITY.addListener(
        (velocity) ->
            yController.setConstraints(
                new TrapezoidProfile.Constraints(
                    velocity, yController.getConstraints().maxAcceleration)));

    ANGLE_MAX_ACCELERATION.addListener(
        (acceleration) ->
            angleController.setConstraints(
                new TrapezoidProfile.Constraints(
                    angleController.getConstraints().maxVelocity, acceleration)));
    POSITION_MAX_ACCELERATION.addListener(
        (acceleration) ->
            xController.setConstraints(
                new TrapezoidProfile.Constraints(
                    xController.getConstraints().maxVelocity, acceleration)));
    POSITION_MAX_ACCELERATION.addListener(
        (acceleration) ->
            yController.setConstraints(
                new TrapezoidProfile.Constraints(
                    yController.getConstraints().maxVelocity, acceleration)));

    // Setup PID controllers
    angleController.enableContinuousInput(-Math.PI, Math.PI);
  }

  // ========== Pose2dSequence Class ==========
  public static final class Pose2dSequence {
    public final Pose2d inner;
    public final Pose2d outer;

    public Pose2dSequence(Pose2d inner, Pose2d outer) {
      this.inner = inner;
      this.outer = outer;
    }

    public static final Pose2dSequence kZero = new Pose2dSequence(Pose2d.kZero, Pose2d.kZero);
  }

  // ========== Helper Methods ==========
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

  // =========================================
  // =========== Get data commands ===========
  // =========================================

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

  private static Pose2d getRadiusStrafePose(
      Translation2d hubCenter,
      Translation2d robotPos,
      double radiusMeters,
      double strafeSpeedRadsPerSecond) {
    Translation2d hubToRobot = robotPos.minus(hubCenter);
    double strafeAngleDelta =
        strafeSpeedRadsPerSecond
            / 5; // divide by robot ticks per second (tick every 20ms, so 5 per second)
    double angleToRobot = Math.atan2(hubToRobot.getY(), hubToRobot.getX()) + strafeAngleDelta;

    Translation2d targetPosition =
        hubCenter.plus(
            new Translation2d(
                radiusMeters * Math.cos(angleToRobot), radiusMeters * Math.sin(angleToRobot)));

    // Heading: face the hub from the target position on the circle.
    // Both targetPosition and hubCenter are fixed at defer time so this angle is
    // perfectly stable — no per-cycle noise.
    // 03/30/26 JITHIN: Inverted angle by adding Pi so that the robot's logical "front" (intake)
    // faces exactly away from the hub
    double angleToHub =
        MathUtil.angleModulus(
            Math.PI
                + Math.atan2(
                    hubCenter.getY() - targetPosition.getY(),
                    hubCenter.getX() - targetPosition.getX()));

    return new Pose2d(targetPosition, new Rotation2d(angleToHub));
  }

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
    // 03/30/26 JITHIN: Inverted angle by adding Pi so that the robot's logical "front" (intake)
    // faces exactly away from the hub
    double angleToHub =
        MathUtil.angleModulus(
            Math.PI
                + Math.atan2(
                    hubCenter.getY() - targetPosition.getY(),
                    hubCenter.getX() - targetPosition.getX()));

    return new Pose2d(targetPosition, new Rotation2d(angleToHub));
  }

  public static boolean isInCircularZone(
      Pose2d robotPose, Translation2d zoneCenter, Distance radius) {
    return robotPose.getTranslation().getDistance(zoneCenter) <= radius.in(Meters);
  }

  public static boolean isInHubZone(Drive drive, Distance radius) {
    return isInCircularZone(drive.getPose(), getHubCenter(), radius);
  }

  public static Translation2d getHubCenter() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
        ? RED_HUB_CENTER
        : BLUE_HUB_CENTER;
  }

  public static Translation2d getLeftVolley() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
        ? new Translation2d(14, 2)
        : new Translation2d(2, 6);
  }

  public static Translation2d getRightVolley() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
        ? new Translation2d(14, 6)
        : new Translation2d(2, 2);
  }

  // ============================================
  // =========== Actual snap commands ===========
  // ============================================
  public static Command snapToClosestPosition(
      Drive drive, Pose2d[] outerPositions, Pose2d[] innerPositions, Distance maxRadius) {
    return Commands.defer(
            () -> {
              Pose2dSequence poses =
                  getClosestPositionSequence(drive, outerPositions, innerPositions, maxRadius)
                      .orElse(Pose2dSequence.kZero);

              return snapToPosition(drive, poses.outer, poses.inner);
            },
            Set.of(drive))
        .withName("SnapCommands.snapToClosestPosition");
  }

  public static Command snapToClosestPositionInterpolation(
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
              return snapToPositionInterpolation(drive, poses.outer, poses.inner, interpolateTime);
            },
            Set.of(drive))
        .withName("SnapCommands.snapToClosestPositionInterpolation");
  }

  public static Command snapToPosition(Drive drive, Pose2d outerPose, Pose2d innerPose) {
    return Commands.defer(
            () -> {
              Logger.recordOutput("Snap/OuterPose", outerPose);
              Logger.recordOutput("Snap/InnerPose", innerPose);

              return snapToPosition(drive, outerPose).andThen(snapToPosition(drive, innerPose));
            },
            Set.of(drive))
        .withName("SnapCommands.snapToPosition.dual");
  }

  public static Command snapToPositionInterpolation(
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
              .withName("SnapCommands.snapToPositionInterpolation");
        },
        Set.of(drive));
  }

  public static Command snapToPosition(Drive drive, Pose2d desiredPosition) {
    return Commands.run(
            () -> {
              var x = xController.calculate(drive.getPose().getX(), desiredPosition.getX());
              var y = yController.calculate(drive.getPose().getY(), desiredPosition.getY());
              var omega =
                  angleController.calculate(
                      drive.getRotation().getRadians(), desiredPosition.getRotation().getRadians());

              Logger.recordOutput("Snap/DesiredPose", desiredPosition);

              ChassisSpeeds speeds = new ChassisSpeeds(x, y, omega);
              drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              var fieldRelativeSpeeds =
                  ChassisSpeeds.fromRobotRelativeSpeeds(
                      drive.getChassisSpeeds(), drive.getPose().getRotation());
              angleController.reset(
                  drive.getRotation().getRadians(), fieldRelativeSpeeds.omegaRadiansPerSecond);
              xController.reset(drive.getPose().getX(), fieldRelativeSpeeds.vxMetersPerSecond);
              yController.reset(drive.getPose().getY(), fieldRelativeSpeeds.vyMetersPerSecond);
            })
        .until(() -> angleController.atGoal() && xController.atGoal() && yController.atGoal())
        .withName("SnapCommands.snapToPosition.single")
        .finallyDo(
            () -> {
              drive.runVelocity(new ChassisSpeeds());
            });
  }

  public static Command snapToRadiusStrafe(
      Drive drive, Distance radius, double StrafeRadsPerSecond) {
    return Commands.defer(
            () -> {
              Translation2d hubCenter = getHubCenter();
              double radiusMeters = radius.in(Meters);
              Translation2d robotPos = drive.getPose().getTranslation();

              Pose2d innerPose = getRadiusTargetPose(hubCenter, robotPos, radiusMeters);

              Logger.recordOutput("SnapToRadius/InnerPose", innerPose);
              Logger.recordOutput("SnapToRadius/DesiredRadius", radiusMeters);

              return snapToPosition(drive, innerPose);
            },
            Set.of(drive))
        .withName("SnapCommands.snapToRadiusStrafe");
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

              return snapToPosition(drive, innerPose);
            },
            Set.of(drive))
        .withName("SnapCommands.snapToRadius");
  }

  public static Command tuneableSnapToRadius(Drive drive) {
    return Commands.defer(
            () -> {
              return snapToRadius(drive, Feet.of(TueableSnapToRadiusFeet.get()));
            },
            Set.of(drive))
        .withName("SnapCommands.snapToRadius");
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

              return snapToPositionInterpolation(drive, outerPose, innerPose, interpolateTime);
            },
            Set.of(drive))
        .withName("SnapCommands.snapToRadiusInterpolation");
  }

  public static Command snapToPosition(Drive drive, Distance x, Distance y, Angle angle) {
    return snapToPosition(
        drive, new Pose2d(new Translation2d(x.in(Meters), y.in(Meters)), new Rotation2d(angle)));
  }

  public static Command snapToAngle(Drive drive, double angleDegrees) {
    return snapToAngle(drive, () -> Rotation2d.fromDegrees(angleDegrees));
  }

  public static Command snapToAngle(Drive drive, Supplier<Rotation2d> angleDegrees) {
    ProfiledPIDController headingController =
        new ProfiledPIDController(
            ANGLE_KP,
            ANGLE_KI,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(
                ANGLE_MAX_VELOCITY.get(), ANGLE_MAX_ACCELERATION.get()));

    headingController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              Rotation2d target = angleDegrees.get();
              double omega =
                  headingController.calculate(
                      drive.getRotation().getRadians(), target.getRadians());

              Logger.recordOutput("SnapToAngle/TargetHeading", target);

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      new ChassisSpeeds(0, 0, omega), drive.getRotation()));
            },
            drive)
        .beforeStarting(
            () -> {
              var fieldRelativeSpeeds =
                  ChassisSpeeds.fromRobotRelativeSpeeds(
                      drive.getChassisSpeeds(), drive.getPose().getRotation());
              headingController.reset(
                  drive.getRotation().getRadians(), fieldRelativeSpeeds.omegaRadiansPerSecond);
            })
        .withName("SnapCommands.snapToAngle");
  }

  // TODO: make this work (prob better to do manualy using overtuned pids)
  //   public static Command swerveJiggle(Drive drive) {
  //     return Commands.repeatingSequence(
  //         snapToAngle(drive, 5).withTimeout(0.2),
  //         Commands.waitSeconds(0.1),
  //         snapToAngle(drive, -5).withTimeout(0.2),
  //         Commands.waitSeconds(0.1));
  //   }

  public static Command snapToAngleIfInZone(
      Drive drive,
      Translation2d zoneCenter,
      Distance zoneRadius,
      double inZoneAngleDegrees,
      double outOfZoneAngleDegrees) {
    return Commands.defer(
        () -> {
          double angleDegrees =
              isInCircularZone(drive.getPose(), zoneCenter, zoneRadius)
                  ? inZoneAngleDegrees
                  : outOfZoneAngleDegrees;
          Logger.recordOutput("SnapToAngleIfInZone/TargetAngleDegrees", angleDegrees);
          return snapToAngle(drive, angleDegrees);
        },
        Set.of(drive));
  }
}
