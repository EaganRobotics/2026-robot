package frc.robot26.util.Ai;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * SpinPathfinder - Just spins in place. Useful for testing that the pathfinder interface works
 * correctly.
 */
public class SpinPathfinder implements AIPathfinder {

  @Override
  public Translation2d nextWaypoint(Pose2d botPose, Pose2d targetPose) {
    return botPose.getTranslation(); // stay in place
  }

  @Override
  public double omegaRadiansPerSecond() {
    return 5.0; // fast spin
  }
}
