package frc.robot26.util.Ai;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * ACOPathfinder - Ant Colony Optimization pathfinding.
 *
 * <p>All bots using this pathfinder share a single pheromone grid. As bots move toward the target
 * they reinforce their paths, causing the swarm to collectively converge on efficient routes over
 * time.
 *
 * <p>Field: 17.55m x 8.02m, grid resolution: 0.5m per cell.
 */
public class ACOPathfinder implements AIPathfinder {

  // ── Field constants ───────────────────────────────────────────────────────
  public static final double FIELD_WIDTH = 17.55;
  public static final double FIELD_HEIGHT = 8.02;
  public static final double CELL_SIZE = 0.50;

  public static final int GRID_COLS = (int) Math.ceil(FIELD_WIDTH / CELL_SIZE);
  public static final int GRID_ROWS = (int) Math.ceil(FIELD_HEIGHT / CELL_SIZE);

  // ── ACO tuning ────────────────────────────────────────────────────────────
  private static final double EVAPORATION_RATE = 0.02;
  private static final double DEPOSIT_AMOUNT = 1.0;
  private static final double MIN_PHEROMONE = 0.1;
  private static final double MAX_PHEROMONE = 10.0;
  private static final double HEURISTIC_WEIGHT = 0.4;

  // ── Shared pheromone grid (all ACOPathfinder instances share this) ────────
  private static final double[][] pheromones = new double[GRID_ROWS][GRID_COLS];

  static {
    for (int r = 0; r < GRID_ROWS; r++)
      for (int c = 0; c < GRID_COLS; c++) pheromones[r][c] = MIN_PHEROMONE;
  }

  // ── AIPathfinder implementation ───────────────────────────────────────────

  @Override
  public Translation2d nextWaypoint(Pose2d botPose, Pose2d targetPose) {
    evaporate();
    logGrid(targetPose);

    int[] currentCell = toCell(botPose.getTranslation());
    int[] targetCell = toCell(targetPose.getTranslation());

    List<int[]> neighbors = getNeighbors(currentCell[0], currentCell[1]);
    if (neighbors.isEmpty()) return targetPose.getTranslation();

    double bestScore = -1;
    int[] bestCell = neighbors.get(0);

    for (int[] neighbor : neighbors) {
      double pheromone = pheromones[neighbor[0]][neighbor[1]];
      double dr = neighbor[0] - targetCell[0];
      double dc = neighbor[1] - targetCell[1];
      double distToTarget = Math.sqrt(dr * dr + dc * dc) + 0.001;
      double heuristic = 1.0 / distToTarget;
      double score = (1.0 - HEURISTIC_WEIGHT) * pheromone + HEURISTIC_WEIGHT * heuristic;
      if (score > bestScore) {
        bestScore = score;
        bestCell = neighbor;
      }
    }

    return toCellCenter(bestCell[0], bestCell[1]);
  }

  @Override
  public void onArrival(Translation2d arrivedAt) {
    int[] cell = toCell(arrivedAt);
    pheromones[cell[0]][cell[1]] =
        Math.min(MAX_PHEROMONE, pheromones[cell[0]][cell[1]] + DEPOSIT_AMOUNT);
  }

  @Override
  public void onStuck() {
    for (int r = 0; r < GRID_ROWS; r++)
      for (int c = 0; c < GRID_COLS; c++) pheromones[r][c] = MIN_PHEROMONE;
  }

  // ── Internal helpers ──────────────────────────────────────────────────────

  private static void evaporate() {
    for (int r = 0; r < GRID_ROWS; r++)
      for (int c = 0; c < GRID_COLS; c++)
        pheromones[r][c] = Math.max(MIN_PHEROMONE, pheromones[r][c] * (1.0 - EVAPORATION_RATE));
  }

  private static int[] toCell(Translation2d t) {
    int col = Math.max(0, Math.min(GRID_COLS - 1, (int) Math.floor(t.getX() / CELL_SIZE)));
    int row = Math.max(0, Math.min(GRID_ROWS - 1, (int) Math.floor(t.getY() / CELL_SIZE)));
    return new int[] {row, col};
  }

  private static Translation2d toCellCenter(int row, int col) {
    return new Translation2d((col + 0.5) * CELL_SIZE, (row + 0.5) * CELL_SIZE);
  }

  private static List<int[]> getNeighbors(int row, int col) {
    List<int[]> neighbors = new ArrayList<>();
    for (int dr = -1; dr <= 1; dr++)
      for (int dc = -1; dc <= 1; dc++) {
        if (dr == 0 && dc == 0) continue;
        int nr = row + dr, nc = col + dc;
        if (nr >= 0 && nr < GRID_ROWS && nc >= 0 && nc < GRID_COLS)
          neighbors.add(new int[] {nr, nc});
      }
    return neighbors;
  }

  private static void logGrid(Pose2d targetPose) {
    Logger.recordOutput("ACOPathfinder/TargetPose", targetPose);
    double[] flat = new double[GRID_ROWS * GRID_COLS];
    for (int r = 0; r < GRID_ROWS; r++)
      for (int c = 0; c < GRID_COLS; c++) flat[r * GRID_COLS + c] = pheromones[r][c];
    Logger.recordOutput("ACOPathfinder/PheromoneGrid", flat);
  }
}
