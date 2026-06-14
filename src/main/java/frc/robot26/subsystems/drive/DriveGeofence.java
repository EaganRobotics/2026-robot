package frc.robot26.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.littletonrobotics.junction.Logger;

final class DriveGeofence {
  private static final boolean ENABLED = true;
  private static final double MIN_X_METERS = 2;
  private static final double MAX_X_METERS = 9;
  private static final double MIN_Y_METERS = 2;
  private static final double MAX_Y_METERS = 5;

  private DriveGeofence() {}

  static ChassisSpeeds constrain(Pose2d pose, ChassisSpeeds robotRelativeSpeeds) {
    if (!ENABLED) {
      Logger.recordOutput("Drive/Geofence/Limited", false);
      return robotRelativeSpeeds;
    }

    ChassisSpeeds fieldRelativeSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, pose.getRotation());
    double vx = fieldRelativeSpeeds.vxMetersPerSecond;
    double vy = fieldRelativeSpeeds.vyMetersPerSecond;

    double lowerX = Math.min(MIN_X_METERS, MAX_X_METERS);
    double upperX = Math.max(MIN_X_METERS, MAX_X_METERS);
    double lowerY = Math.min(MIN_Y_METERS, MAX_Y_METERS);
    double upperY = Math.max(MIN_Y_METERS, MAX_Y_METERS);

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
    Logger.recordOutput("Drive/Geofence/Enabled", ENABLED);
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
    double lowerX = Math.min(MIN_X_METERS, MAX_X_METERS);
    double upperX = Math.max(MIN_X_METERS, MAX_X_METERS);
    double lowerY = Math.min(MIN_Y_METERS, MAX_Y_METERS);
    double upperY = Math.max(MIN_Y_METERS, MAX_Y_METERS);

    return !ENABLED
        || (pose.getX() >= lowerX
            && pose.getX() <= upperX
            && pose.getY() >= lowerY
            && pose.getY() <= upperY);
  }
}
