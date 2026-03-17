package frc.robot26.subsystems.intake;

import org.littletonrobotics.junction.Logger;

/**
 * RobotGamePieceStorage - tracks how many game pieces are stored in the robot.
 *
 * <p>IntakeIOSim increments this when a ball is picked up. ShooterIOSim checks and decrements this
 * when firing.
 */
public class RobotGamePieceStorage {

  private static int storedBalls = 10;
  private static final int MAX_BALLS = 20; // adjust to match your robot

  public static int getStoredBalls() {
    return storedBalls;
  }

  public static boolean hasBalls() {
    return storedBalls > 0;
  }

  public static void addBall() {
    storedBalls = Math.min(MAX_BALLS, storedBalls + 1);
    log();
  }

  public static void removeBall() {
    storedBalls = Math.max(0, storedBalls - 1);
    log();
  }

  public static void reset() {
    storedBalls = 0;
    log();
  }

  private static void log() {
    Logger.recordOutput("RobotStorage/StoredBalls", storedBalls);
  }
}
