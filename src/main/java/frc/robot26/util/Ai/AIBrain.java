package frc.robot26.util.Ai;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * AIBrain - Shared state for all AI robots.
 *
 * <p>Holds the player robot's current pose so all pathfinders can access it. Call {@link
 * #update(Pose2d)} every simulationPeriodic from RobotContainer.
 */
public class AIBrain {

  private static Pose2d targetPose = new Pose2d();

  /** Update the shared target pose. Call this every simulationPeriodic with drive.getPose(). */
  public static void update(Pose2d playerRobotPose) {
    targetPose = playerRobotPose;
  }

  /** Returns the current player robot pose. */
  public static Pose2d getTargetPose() {
    return targetPose;
  }
}
