package frc.robot26.util.Ai;

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
  static final Pose2d[] ROBOT_QUEENING_POSITIONS =
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
  private static final double STUCK_VELOCITY_THRESHOLD = 0.2; // m/s
  private static final int STUCK_CONFIRM_LOOPS = 25; // ~0.5s at 50hz
  private static final int ESCAPE_HOLD_LOOPS = 60; // ~1.2s at 50hz
  private static final int STARTUP_GRACE_LOOPS = 100; // ~2s at 50hz
  public static final double WAYPOINT_ARRIVAL_THRESHOLD = 0.4; // meters

  // ── Static instances ──────────────────────────────────────────────────────
  private static final AIRobotInSimulation[] instances = new AIRobotInSimulation[3];
  private static final Random random = new Random();

  /**
   * Start all opponent robot simulations. Each bot gets its own pathfinder — swap them out here to
   * change behavior.
   */
  public static void startOpponentRobotSimulations() {
    System.out.println("Starting opponent robot simulations...");
    instances[0] = new AIRobotInSimulation(0, new ACOPathfinder());
    instances[1] = new AIRobotInSimulation(1, new SpinPathfinder());
    instances[2] = new AIRobotInSimulation(2, new RandomPathfinder());
  }

  // ── Instance fields ───────────────────────────────────────────────────────
  private final SwerveDriveSimulation driveSimulation;
  private final AIPathfinder pathfinder;
  private final Pose2d spawnPose;
  private final int id;

  private Translation2d currentWaypoint;
  private Translation2d escapeDirection = null;

  private int loopCount = 0;
  private int slowLoopCount = 0;
  private int escapeLoopCount = 0;

  public AIRobotInSimulation(int id, AIPathfinder pathfinder) {
    this.id = id;
    this.pathfinder = pathfinder;
    this.spawnPose = ROBOT_QUEENING_POSITIONS[id];
    this.driveSimulation = new SwerveDriveSimulation(DRIVETRAIN_CONFIG, spawnPose);
    this.currentWaypoint = spawnPose.getTranslation();

    SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
  }

  // ── Periodic ──────────────────────────────────────────────────────────────

  @Override
  public void periodic() {
    loopCount++;
    Pose2d currentPose = driveSimulation.getSimulatedDriveTrainPose();
    Pose2d targetPose = AIBrain.getTargetPose();

    // ── Stuck detection (skip during startup grace period) ────────────────
    if (loopCount > STARTUP_GRACE_LOOPS) {
      if (escapeDirection != null) {
        escapeLoopCount++;
        if (escapeLoopCount >= ESCAPE_HOLD_LOOPS) {
          escapeDirection = null;
          escapeLoopCount = 0;
          slowLoopCount = 0;
        }
      } else {
        ChassisSpeeds fieldSpeeds =
            driveSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
        double speed = Math.hypot(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

        if (speed < STUCK_VELOCITY_THRESHOLD) {
          slowLoopCount++;
          if (slowLoopCount >= STUCK_CONFIRM_LOOPS) {
            double angle = random.nextDouble() * 2.0 * Math.PI;
            escapeDirection = new Translation2d(Math.cos(angle), Math.sin(angle));
            escapeLoopCount = 0;
            slowLoopCount = 0;
            pathfinder.onStuck();
          }
        } else {
          slowLoopCount = 0;
        }
      }
    }

    // ── Drive logic ───────────────────────────────────────────────────────
    if (escapeDirection != null) {
      driveSimulation.setRobotSpeeds(
          ChassisSpeeds.fromFieldRelativeSpeeds(
              escapeDirection.getX() * MAX_DRIVE_SPEED,
              escapeDirection.getY() * MAX_DRIVE_SPEED,
              0.0,
              currentPose.getRotation()));
      Logger.recordOutput("AIRobots/Robot" + id + "/Stuck", true);
    } else {
      double distToWaypoint = currentPose.getTranslation().getDistance(currentWaypoint);
      if (distToWaypoint < WAYPOINT_ARRIVAL_THRESHOLD) {
        pathfinder.onArrival(currentPose.getTranslation());
        currentWaypoint = pathfinder.nextWaypoint(currentPose, targetPose);
      }
      driveToward(currentPose, currentWaypoint);
      Logger.recordOutput("AIRobots/Robot" + id + "/Stuck", false);
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
      driveSimulation.setRobotSpeeds(new ChassisSpeeds(0, 0, pathfinder.omegaRadiansPerSecond()));
      return;
    }

    double speed = Math.min(MAX_DRIVE_SPEED, distance * 2.0);
    double vx = (delta.getX() / distance) * speed;
    double vy = (delta.getY() / distance) * speed;

    driveSimulation.setRobotSpeeds(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            vx, vy, pathfinder.omegaRadiansPerSecond(), currentPose.getRotation()));
  }
}
