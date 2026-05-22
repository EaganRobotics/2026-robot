package frc.robot26.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.lib.tunables.LoggedTunableBoolean;
import frc.lib.tunables.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

final class DriveGeofence {
  private static final double ROBOT_CENTER_MARGIN_METERS = 0.42;

  private static final LoggedTunableBoolean enabled =
      new LoggedTunableBoolean("Tuning/DriveGeofence/Enabled", true);
  private static final LoggedTunableNumber minX =
      new LoggedTunableNumber("Tuning/DriveGeofence/MinX", ROBOT_CENTER_MARGIN_METERS);
  private static final LoggedTunableNumber maxX =
      new LoggedTunableNumber("Tuning/DriveGeofence/MaxX", 16.54 - ROBOT_CENTER_MARGIN_METERS);
  private static final LoggedTunableNumber minY =
      new LoggedTunableNumber("Tuning/DriveGeofence/MinY", ROBOT_CENTER_MARGIN_METERS);
  private static final LoggedTunableNumber maxY =
      new LoggedTunableNumber("Tuning/DriveGeofence/MaxY", 8.07 - ROBOT_CENTER_MARGIN_METERS);

  private DriveGeofence() {}

  static ChassisSpeeds constrain(Pose2d pose, ChassisSpeeds robotRelativeSpeeds) {
    if (!enabled.get()) {
      Logger.recordOutput("Drive/Geofence/Limited", false);
      return robotRelativeSpeeds;
    }

    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, pose.getRotation());
    double vx = fieldRelativeSpeeds.vxMetersPerSecond;
    double vy = fieldRelativeSpeeds.vyMetersPerSecond;

    double lowerX = Math.min(minX.get(), maxX.get());
    double upperX = Math.max(minX.get(), maxX.get());
    double lowerY = Math.min(minY.get(), maxY.get());
    double upperY = Math.max(minY.get(), maxY.get());

    if (pose.getX() <= lowerX && vx < 0.0) {
      vx = 0.0;
    } else if (pose.getX() >= upperX && vx > 0.0) {
      vx = 0.0;
    }

    if (pose.getY() <= lowerY && vy < 0.0) {
      vy = 0.0;
    } else if (pose.getY() >= upperY && vy > 0.0) {
      vy = 0.0;
    }

    boolean limited =
        vx != fieldRelativeSpeeds.vxMetersPerSecond || vy != fieldRelativeSpeeds.vyMetersPerSecond;
    Logger.recordOutput("Drive/Geofence/Limited", limited);
    Logger.recordOutput("Drive/Geofence/Inside", isInside(pose));
    Logger.recordOutput("Drive/Geofence/MinX", lowerX);
    Logger.recordOutput("Drive/Geofence/MaxX", upperX);
    Logger.recordOutput("Drive/Geofence/MinY", lowerY);
    Logger.recordOutput("Drive/Geofence/MaxY", upperY);

    if (!limited) {
      return robotRelativeSpeeds;
    }

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        new ChassisSpeeds(vx, vy, fieldRelativeSpeeds.omegaRadiansPerSecond), pose.getRotation());
  }

  static boolean isInside(Pose2d pose) {
    double lowerX = Math.min(minX.get(), maxX.get());
    double upperX = Math.max(minX.get(), maxX.get());
    double lowerY = Math.min(minY.get(), maxY.get());
    double upperY = Math.max(minY.get(), maxY.get());

    return !enabled.get()
        || (pose.getX() >= lowerX
            && pose.getX() <= upperX
            && pose.getY() >= lowerY
            && pose.getY() <= upperY);
  }
}
