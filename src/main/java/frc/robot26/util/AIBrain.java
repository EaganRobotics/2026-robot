package frc.robot26.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * AIBrain - Shared Ant Colony Optimization (ACO) pathfinding engine.
 *
 * <p>All AI robots share a single pheromone grid. As bots successfully move toward the target, they
 * reinforce the paths they took. Pheromones evaporate over time, causing the swarm to collectively
 * converge on efficient routes to chase/defend against the player robot.
 *
 * <p>Field dimensions: 17.55m x 8.02m (2026 FRC field) Grid resolution: 0.5m per cell
 */
public class AIBrain {

  // ── Field constants ───────────────────────────────────────────────────────
  public static final double FIELD_WIDTH = 17.55; // meters
  public static final double FIELD_HEIGHT = 8.02; // meters
  public static final double CELL_SIZE = 0.50; // meters per grid cell

  public static final int GRID_COLS = (int) Math.ceil(FIELD_WIDTH / CELL_SIZE); // 36
  public static final int GRID_ROWS = (int) Math.ceil(FIELD_HEIGHT / CELL_SIZE); // 17

  // ── ACO tuning ────────────────────────────────────────────────────────────
  /** How much pheromone evaporates each periodic call (0–1). */
  private static final double EVAPORATION_RATE = 0.02;

  /** Pheromone deposited when a bot successfully moves closer to target. */
  private static final double DEPOSIT_AMOUNT = 1.0;

  /** Minimum pheromone floor so all cells stay explorable. */
  private static final double MIN_PHEROMONE = 0.1;

  /** Maximum pheromone ceiling. */
  private static final double MAX_PHEROMONE = 10.0;

  /**
   * Weight blending heuristic (distance-to-target) vs pheromone. 0 = pure pheromone, 1 = pure
   * heuristic (greedy).
   */
  private static final double HEURISTIC_WEIGHT = 0.4;

  /** How close a bot needs to be to its current waypoint before picking the next one (meters). */
  public static final double WAYPOINT_ARRIVAL_THRESHOLD = 0.4;

  // ── Shared pheromone grid (row-major: [row][col]) ─────────────────────────
  private static final double[][] pheromones = new double[GRID_ROWS][GRID_COLS];

  // ── Target pose (your robot) ──────────────────────────────────────────────
  private static Pose2d targetPose = new Pose2d();

  static {
    // Initialize all pheromones to the floor value
    for (int r = 0; r < GRID_ROWS; r++)
      for (int c = 0; c < GRID_COLS; c++) pheromones[r][c] = MIN_PHEROMONE;
  }

  // ── Public API ────────────────────────────────────────────────────────────

  /**
   * Call once per robot periodic with the player robot's current pose. Also runs pheromone
   * evaporation and logs the grid.
   */
  public static void update(Pose2d playerRobotPose) {
    targetPose = playerRobotPose;
    evaporate();
    logGrid();
  }

  /**
   * Given a bot's current pose, returns the next Translation2d waypoint it should drive toward
   * using ACO + heuristic blending.
   *
   * <p>After calling this, the bot should call {@link #reinforce} once it arrives at the returned
   * waypoint.
   */
  public static Translation2d nextWaypoint(Pose2d botPose) {
    int[] currentCell = toCell(botPose.getTranslation());
    int[] targetCell = toCell(targetPose.getTranslation());

    // Get all valid 8-directional neighbors
    List<int[]> neighbors = getNeighbors(currentCell[0], currentCell[1]);

    if (neighbors.isEmpty()) {
      // Stuck in a corner — just head straight for the target
      return targetPose.getTranslation();
    }

    // Score each neighbor: blend pheromone strength with distance-to-target heuristic
    double bestScore = -1;
    int[] bestCell = neighbors.get(0);

    for (int[] neighbor : neighbors) {
      double pheromone = pheromones[neighbor[0]][neighbor[1]];

      // Heuristic: inverse of distance from this neighbor to the target cell
      double dr = neighbor[0] - targetCell[0];
      double dc = neighbor[1] - targetCell[1];
      double distToTarget = Math.sqrt(dr * dr + dc * dc) + 0.001; // avoid /0
      double heuristic = 1.0 / distToTarget;

      double score = (1.0 - HEURISTIC_WEIGHT) * pheromone + HEURISTIC_WEIGHT * heuristic;

      if (score > bestScore) {
        bestScore = score;
        bestCell = neighbor;
      }
    }

    return toPose(bestCell[0], bestCell[1]);
  }

  /**
   * Call when a bot successfully moves closer to the target. Deposits pheromone at the cell the bot
   * just came from, reinforcing that path.
   */
  public static void reinforce(Translation2d arrivedAt) {
    int[] cell = toCell(arrivedAt);
    pheromones[cell[0]][cell[1]] =
        Math.min(MAX_PHEROMONE, pheromones[cell[0]][cell[1]] + DEPOSIT_AMOUNT);
  }

  /** Returns the current target pose (player robot). */
  public static Pose2d getTargetPose() {
    return targetPose;
  }

  // ── Internal helpers ──────────────────────────────────────────────────────

  /** Evaporate all pheromones by EVAPORATION_RATE, clamped to MIN_PHEROMONE. */
  private static void evaporate() {
    for (int r = 0; r < GRID_ROWS; r++) {
      for (int c = 0; c < GRID_COLS; c++) {
        pheromones[r][c] = Math.max(MIN_PHEROMONE, pheromones[r][c] * (1.0 - EVAPORATION_RATE));
      }
    }
  }

  /** Convert a field Translation2d to a [row, col] grid cell, clamped to grid bounds. */
  public static int[] toCell(Translation2d translation) {
    int col = (int) Math.floor(translation.getX() / CELL_SIZE);
    int row = (int) Math.floor(translation.getY() / CELL_SIZE);
    col = Math.max(0, Math.min(GRID_COLS - 1, col));
    row = Math.max(0, Math.min(GRID_ROWS - 1, row));
    return new int[] {row, col};
  }

  /** Convert a [row, col] grid cell back to the center Translation2d of that cell. */
  public static Translation2d toPose(int row, int col) {
    double x = (col + 0.5) * CELL_SIZE;
    double y = (row + 0.5) * CELL_SIZE;
    return new Translation2d(x, y);
  }

  /** Resets all pheromones back to the floor value. Called when a bot gets stuck. */
  public static void resetPheromones() {
    for (int r = 0; r < GRID_ROWS; r++)
      for (int c = 0; c < GRID_COLS; c++) pheromones[r][c] = MIN_PHEROMONE;
  }

  /** Return all valid 8-directional neighbors for a given cell. */
  private static List<int[]> getNeighbors(int row, int col) {
    List<int[]> neighbors = new ArrayList<>();
    for (int dr = -1; dr <= 1; dr++) {
      for (int dc = -1; dc <= 1; dc++) {
        if (dr == 0 && dc == 0) continue;
        int nr = row + dr;
        int nc = col + dc;
        if (nr >= 0 && nr < GRID_ROWS && nc >= 0 && nc < GRID_COLS) {
          neighbors.add(new int[] {nr, nc});
        }
      }
    }
    return neighbors;
  }

  /** Log the pheromone grid and target pose to AdvantageKit for visualization. */
  private static void logGrid() {
    // Log target
    Logger.recordOutput("AIBrain/TargetPose", targetPose);

    // Flatten grid to double[] for logging
    double[] flat = new double[GRID_ROWS * GRID_COLS];
    for (int r = 0; r < GRID_ROWS; r++)
      for (int c = 0; c < GRID_COLS; c++) flat[r * GRID_COLS + c] = pheromones[r][c];

    Logger.recordOutput("AIBrain/PheromoneGrid", flat);
    Logger.recordOutput("AIBrain/GridRows", (double) GRID_ROWS);
    Logger.recordOutput("AIBrain/GridCols", (double) GRID_COLS);
    Logger.recordOutput("AIBrain/CellSize", CELL_SIZE);
  }
}
