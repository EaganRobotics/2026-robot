package frc.robot26.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot26.generated.TunerConstants;
import java.util.Random;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.Logger;

public class AIRobotInSimulation extends SubsystemBase {

  // ── Spawn positions (center of field) ─────────────────────────────────────
  public static final Pose2d[] ROBOT_QUEENING_POSITIONS =
      new Pose2d[] {
        new Pose2d(8.0, 4.0, new Rotation2d()),
        new Pose2d(9.0, 4.0, new Rotation2d()),
        new Pose2d(10.0, 4.0, new Rotation2d()),
        new Pose2d(8.0, 3.0, new Rotation2d()),
        new Pose2d(9.0, 3.0, new Rotation2d())
      };

  // ── Drivetrain simulation config ──────────────────────────────────────────
  private static final DriveTrainSimulationConfig DRIVETRAIN_CONFIG =
      DriveTrainSimulationConfig.Default()
          .withRobotMass(Kilograms.of(55))
          .withTrackLengthTrackWidth(Inches.of(10.875 * 2), Inches.of(10.875 * 2))
          .withBumperSize(Inches.of(30), Inches.of(30))
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  DCMotor.getKrakenX60(1),
                  DCMotor.getFalcon500(1),
                  TunerConstants.kDriveGearRatio,
                  26.0,
                  Volts.of(0.2),
                  Volts.of(0.2),
                  Inches.of(2),
                  KilogramSquareMeters.of(0.00001),
                  1.2));

  // ── Tuning ────────────────────────────────────────────────────────────────
  private static final double MAX_DRIVE_SPEED = 3.0;

  /** Loops between stuck checks (~50 = 1 second at 50hz). */
  private static final int STUCK_CHECK_INTERVAL = 50;

  /** Movement threshold to be considered stuck (meters). */
  private static final double STUCK_THRESHOLD_METERS = 0.1;

  // ── Static instances ──────────────────────────────────────────────────────
  public static final AIRobotInSimulation[] instances = new AIRobotInSimulation[3];
  private static final Random random = new Random();

  public static void startOpponentRobotSimulations() {
    System.out.println("Starting opponent robot simulations...");
    instances[0] = new AIRobotInSimulation(0);
    instances[1] = new AIRobotInSimulation(1);
    instances[2] = new AIRobotInSimulation(2);
  }

  // ── Instance fields ───────────────────────────────────────────────────────
  private final SwerveDriveSimulation driveSimulation;
  private final Pose2d spawnPose;
  private final int id;

  private Translation2d currentWaypoint;
  private Translation2d lastCheckedPosition;
  private int stuckCheckCounter = 0;

  /** Random escape direction when stuck, null when not stuck. */
  private Translation2d escapeDirection = null;

  public AIRobotInSimulation(int id) {
    this.id = id;
    this.spawnPose = ROBOT_QUEENING_POSITIONS[id];
    this.driveSimulation = new SwerveDriveSimulation(DRIVETRAIN_CONFIG, spawnPose);
    this.currentWaypoint = spawnPose.getTranslation();
    this.lastCheckedPosition = spawnPose.getTranslation();

    SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
  }

  // ── Periodic ──────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    Pose2d currentPose = driveSimulation.getSimulatedDriveTrainPose();

    // ── Stuck detection ───────────────────────────────────────────────────
    stuckCheckCounter++;
    if (stuckCheckCounter >= STUCK_CHECK_INTERVAL) {
      stuckCheckCounter = 0;
      double moved = currentPose.getTranslation().getDistance(lastCheckedPosition);

      if (moved < STUCK_THRESHOLD_METERS) {
        // Pick a new random escape direction if we don't already have one
        if (escapeDirection == null) {
          double angle = random.nextDouble() * 2.0 * Math.PI;
          escapeDirection = new Translation2d(Math.cos(angle), Math.sin(angle));
        }
        Logger.recordOutput("AIRobots/Robot" + id + "/Stuck", true);
      } else {
        // We've moved — clear escape mode and resume ACO
        escapeDirection = null;
        Logger.recordOutput("AIRobots/Robot" + id + "/Stuck", false);
      }

      lastCheckedPosition = currentPose.getTranslation();
    }

    // ── Drive logic ───────────────────────────────────────────────────────
    if (escapeDirection != null) {
      // Drive in the random escape direction until we're unstuck
      driveSimulation.setRobotSpeeds(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              escapeDirection.getX() * MAX_DRIVE_SPEED,
              escapeDirection.getY() * MAX_DRIVE_SPEED,
              0.0,
              currentPose.getRotation()));
    } else {
      // Normal ACO pathfinding
      double distToWaypoint = currentPose.getTranslation().getDistance(currentWaypoint);
      if (distToWaypoint < AIBrain.WAYPOINT_ARRIVAL_THRESHOLD) {
        AIBrain.reinforce(currentPose.getTranslation());
        currentWaypoint = AIBrain.nextWaypoint(currentPose);
      }
      driveToward(currentPose, currentWaypoint);
    }

    Logger.recordOutput("AIRobots/Robot" + id + "/Pose", currentPose);
    Logger.recordOutput(
        "AIRobots/Robot" + id + "/Waypoint", new Pose2d(currentWaypoint, new Rotation2d()));
  }

  // ── Internal drive logic ──────────────────────────────────────────────────

  private void driveToward(Pose2d currentPose, Translation2d target) {
    Translation2d delta = target.minus(currentPose.getTranslation());
    double distance = delta.getNorm();

    if (distance < 0.05) {
      driveSimulation.setRobotSpeeds(new ChassisSpeeds());
      return;
    }

    double speed = Math.min(MAX_DRIVE_SPEED, distance * 2.0);
    double vx = (delta.getX() / distance) * speed;
    double vy = (delta.getY() / distance) * speed;

    driveSimulation.setRobotSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, 0.0, currentPose.getRotation()));
  }
}
