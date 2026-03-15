package frc.robot26.util.Ai;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * AIPathfinder - Interface for AI robot pathfinding algorithms.
 *
 * <p>Implement this to create a new pathfinding strategy for AI robots. Each AIRobotInSimulation
 * takes one of these, so each bot can run a different algorithm.
 *
 * <p>Implementations in this package: - {@link ACOPathfinder} - Ant Colony Optimization (shared
 * pheromone grid, chases player) - {@link RandomPathfinder} - Drives to random field positions
 * (useful for testing/variety)
 */
public interface AIPathfinder {

  /**
   * Called every periodic loop. Returns the next Translation2d waypoint the bot should drive
   * toward.
   *
   * @param botPose the bot's current pose in the simulation world
   * @param targetPose the player robot's current pose
   * @return the next waypoint to drive toward
   */
  Translation2d nextWaypoint(Pose2d botPose, Pose2d targetPose);

  /**
   * Called when the bot successfully arrives at a waypoint. Use this to update internal state (e.g.
   * pheromone reinforcement).
   *
   * @param arrivedAt the position the bot arrived at
   */
  default void onArrival(Translation2d arrivedAt) {}

  /** Called when the bot is detected as stuck. Use this to reset internal state if needed. */
  default void onStuck() {}

  default double omegaRadiansPerSecond() {
    return 0.0;
  }
}
