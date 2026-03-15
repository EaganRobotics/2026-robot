package frc.robot26.util.Ai;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Random;

/**
 * RandomPathfinder - Drives to random positions on the field.
 *
 * <p>Simple but unpredictable. Good as a second algorithm to mix with ACO so not all bots behave
 * identically.
 */
public class RandomPathfinder implements AIPathfinder {

  private static final double FIELD_WIDTH = 17.55;
  private static final double FIELD_HEIGHT = 8.02;

  private final Random random = new Random();

  @Override
  public Translation2d nextWaypoint(Pose2d botPose, Pose2d targetPose) {
    // Pick a completely random point on the field
    double x = random.nextDouble() * FIELD_WIDTH;
    double y = random.nextDouble() * FIELD_HEIGHT;
    return new Translation2d(x, y);
  }
}
