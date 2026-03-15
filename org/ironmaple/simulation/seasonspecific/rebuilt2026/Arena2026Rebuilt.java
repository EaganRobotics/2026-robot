package org.ironmaple.simulation.seasonspecific.rebuilt2026;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static org.ironmaple.utils.FieldMirroringUtils.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;
import org.dyn4j.dynamics.Settings;
import org.ironmaple.simulation.SimulatedArena;

public class Arena2026Rebuilt extends SimulatedArena {

    protected boolean shouldClock = true;

    protected double clock = 0;
    protected boolean blueIsOnClock = Math.random() < 0.5;

    protected DoublePublisher phaseClockPublisher =
            genericInfoTable.getDoubleTopic("Time left in current phase").publish();

    protected BooleanPublisher redActivePublisher =
            redTable.getBooleanTopic("Red is active").publish();
    protected BooleanPublisher blueActivePublisher =
            blueTable.getBooleanTopic("Blue is active").publish();

    protected RebuiltHub blueHub;
    protected RebuiltHub redHub;

    protected RebuiltOutpost blueOutpost;
    protected RebuiltOutpost redOutpost;

    protected static Translation2d centerPieceBottomRightCorner = new Translation2d(7.35737, 1.724406);
    protected static Translation2d redDepotBottomRightCorner = new Translation2d(0.02, 5.53);
    protected static Translation2d blueDepotBottomRightCorner = new Translation2d(16.0274, 1.646936);

    public static final class RebuiltFieldObstacleMap extends FieldMap {

        public RebuiltFieldObstacleMap() {
            super();

            // blue wall
            super.addBorderLine(new Translation2d(0, 0), new Translation2d(0, FIELD_HEIGHT));

            // red wall
            super.addBorderLine(new Translation2d(FIELD_WIDTH, 0), new Translation2d(FIELD_WIDTH, FIELD_HEIGHT));

            // side walls
            super.addBorderLine(new Translation2d(0, 0), new Translation2d(FIELD_WIDTH, 0));
            super.addBorderLine(new Translation2d(0, FIELD_HEIGHT), new Translation2d(FIELD_WIDTH, FIELD_HEIGHT));

            var blueTrenchRightPos = new Translation2d(4.626, 1.431);
            var blueTrenchLeftPos = new Translation2d(4.626, FIELD_HEIGHT - blueTrenchRightPos.getY());
            var redTrenchLeftPos = flip(blueTrenchRightPos);
            var redTrenchRightPos = flip(blueTrenchLeftPos);

            // blue hub
            super.addRectangularObstacle(
                    1.19, 1.19, new Pose2d(RebuiltHub.BLUE_HUB_POS.toTranslation2d(), Rotation2d.kZero));

            // blue trench barrier right
            super.addRectangularObstacle(1.194, 0.305, new Pose2d(blueTrenchRightPos, Rotation2d.kZero));

            // blue trench barrier left
            super.addRectangularObstacle(1.19, 0.305, new Pose2d(blueTrenchLeftPos, Rotation2d.kZero));

            // blue tower poles
            super.addRectangularObstacle(
                    Inches.of(2).in(Meters),
                    Inches.of(47).in(Meters),
                    new Pose2d(new Translation2d(Inches.of(42), Inches.of(159)), new Rotation2d()));

            // red hub
            super.addRectangularObstacle(
                    1.19, 1.19, new Pose2d(RebuiltHub.RED_HUB_POS.toTranslation2d(), Rotation2d.kZero));

            // red trench barrier right
            super.addRectangularObstacle(1.19, 0.305, new Pose2d(redTrenchRightPos, Rotation2d.kZero));

            // red trench barrier left
            super.addRectangularObstacle(1.19, 0.305, new Pose2d(redTrenchLeftPos, Rotation2d.kZero));

            // red tower poles
            super.addRectangularObstacle(
                    Inches.of(2).in(Meters),
                    Inches.of(47).in(Meters),
                    new Pose2d(new Translation2d(Inches.of(651 - 42), Inches.of(170)), new Rotation2d()));
        }
    }

    /*
     * field element simulation inits
     */

    public Arena2026Rebuilt() {
        super(new RebuiltFieldObstacleMap());

        Settings settings = physicsWorld.getSettings();

        // settings.setVelocityConstraintSolverIterations(3);
        // settings.setPositionConstraintSolverIterations(2);
        settings.setMinimumAtRestTime(0.02);

        physicsWorld.setSettings(settings);

        blueHub = new RebuiltHub(this, true);
        super.addCustomSimulation(blueHub);

        redHub = new RebuiltHub(this, false);
        super.addCustomSimulation(redHub);

        blueOutpost = new RebuiltOutpost(this, true);
        super.addCustomSimulation(blueOutpost);

        redOutpost = new RebuiltOutpost(this, false);
        super.addCustomSimulation(redOutpost);
    }

    private Translation2d fuelZoneCenter = new Translation2d(FIELD_WIDTH / 2, FIELD_HEIGHT / 2);
    private int neutralFuelCount = 408;
    private double fuelSeparationGap = 0.001; // 1mm

    public void setNeutralFuelCount(int count) {
        this.neutralFuelCount = Math.min(600, count);
    }

    public RebuiltHub getBlueHub() {
        return blueHub;
    }

    public RebuiltHub getRedHub() {
        return redHub;
    }

    public RebuiltOutpost getBlueOutpost() {
        return blueOutpost;
    }

    public RebuiltOutpost getRedOutpost() {
        return redOutpost;
    }

    /**
     *
     *
     * <h2>Adds a game piece too the arena with a certain random variance.</h2>
     *
     * This method is useful for certain spawners like the return cutes on the hub to prevent the game pieces from being
     * returned to the exact same position every time.
     *
     * @param info the info of the game piece
     * @param robotPosition the position of the robot (not the shooter) at the time of launching the game piece
     * @param shooterPositionOnRobot the translation from the shooter's position to the robot's center, in the robot's
     *     frame of reference
     * @param chassisSpeedsFieldRelative the field-relative velocity of the robot chassis when launching the game piece,
     *     influencing the initial velocity of the game piece
     * @param shooterFacing the direction in which the shooter is facing at launch
     * @param initialHeight the initial height of the game piece when launched, i.e., the height of the shooter from the
     *     ground
     * @param launchingSpeed the speed at which the game piece is launch
     * @param shooterAngle the pitch angle of the shooter when launching
     * @param xVariance The max amount of variance that should be added too the x coordinate of the game piece.
     * @param yVariance The max amount of variance that should be added too the y coordinate of the game piece.
     * @param yawVariance The max amount of variance that should be added too the yaw of the game piece.
     * @param speedVariance The max amount of variance that should be added too the speed of the game piece.
     * @param pitchVariance The max amount of variance that should be added too the pitch of the game piece.
     */
    public void addPieceWithVariance(
            Translation2d piecePose,
            Rotation2d yaw,
            Distance height,
            LinearVelocity speed,
            Angle pitch,
            double xVariance,
            double yVariance,
            double yawVariance,
            double speedVariance,
            double pitchVariance) {
        var projectile = new RebuiltFuelOnFly(
                piecePose.plus(new Translation2d(randomInRange(xVariance), randomInRange(yVariance))),
                new Translation2d(),
                new ChassisSpeeds(),
                yaw.plus(Rotation2d.fromDegrees(randomInRange(yawVariance))),
                height,
                speed.plus(MetersPerSecond.of(randomInRange(speedVariance))),
                Degrees.of(pitch.in(Degrees) + randomInRange(pitchVariance)));
        getGamePieceManager().spawnInFlight(projectile);
        projectile.launch();
    }

    @Override
    public void placeGamePiecesOnField() {
        double fuelDiameter =
                ((org.dyn4j.geometry.Circle) RebuiltFuelOnField.REBUILT_FUEL_INFO.shape()).getRadius() * 2;
        double fuelRadius = fuelDiameter / 2;
        double spacing = fuelDiameter + fuelSeparationGap;

        // Dimensions from graphic
        double boundingBoxWidth = edu.wpi.first.units.Units.Inches.of(206).in(edu.wpi.first.units.Units.Meters);
        double boundingBoxDepth = edu.wpi.first.units.Units.Inches.of(72).in(edu.wpi.first.units.Units.Meters);

        java.util.List<Translation2d> q1 = new java.util.ArrayList<>();
        java.util.List<Translation2d> q2 = new java.util.ArrayList<>();
        java.util.List<Translation2d> q3 = new java.util.ArrayList<>();
        java.util.List<Translation2d> q4 = new java.util.ArrayList<>();

        // Generate points for one quadrant (Q1) starting from the center outward
        // We want uniform spacing of 'fuelSeparationGap' between all pieces, including
        // across the axes.
        // So the first piece center should be at (radius + gap/2) from the axis.
        double dividerWidth = edu.wpi.first.units.Units.Inches.of(2).in(edu.wpi.first.units.Units.Meters);
        double startX = dividerWidth / 2 + fuelSeparationGap + fuelRadius;
        double startY = dividerWidth / 2 + fuelSeparationGap + fuelRadius;

        for (double x = startX; x + fuelRadius <= boundingBoxDepth / 2; x += spacing) {
            for (double y = startY; y + fuelRadius <= boundingBoxWidth / 2; y += spacing) {
                // Add mirrored positions to all quadrants
                q1.add(fuelZoneCenter.plus(new Translation2d(x, y))); // Q1
                q2.add(fuelZoneCenter.plus(new Translation2d(-x, y))); // Q2
                q3.add(fuelZoneCenter.plus(new Translation2d(-x, -y))); // Q3
                q4.add(fuelZoneCenter.plus(new Translation2d(x, -y))); // Q4
            }
        }

        // Round robin placement
        java.util.List<java.util.List<Translation2d>> quadrants = java.util.List.of(q1, q2, q3, q4);
        int added = 0;
        boolean piecesAvailable = true;

        // Sort lists to ensure deterministic deterministic filling order (e.g. from
        // center out)
        // Distance to center ascending
        java.util.Comparator<Translation2d> distComparator =
                java.util.Comparator.comparingDouble(p -> p.getDistance(fuelZoneCenter));
        for (java.util.List<Translation2d> q : quadrants) {
            q.sort(distComparator);
        }

        // Indices for each quadrant list
        int[] indices = new int[4];

        while (added < neutralFuelCount && piecesAvailable) {
            piecesAvailable = false;
            for (int i = 0; i < 4; i++) {
                if (added >= neutralFuelCount) break;

                java.util.List<Translation2d> q = quadrants.get(i);
                if (indices[i] < q.size()) {
                    getGamePieceManager().spawnOnField(new RebuiltFuelOnField(q.get(indices[i])));
                    indices[i]++;
                    added++;
                    piecesAvailable = true;
                }
            }
        }

        // Depot Placement
        // Origin: (0.071, 5.584)
        // 4 rows in +y, 6 cols in +x
        Translation2d depotOrigin = new Translation2d(0.071, 5.584);
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 6; j++) {
                double x = depotOrigin.getX() + (i * spacing);
                double y = depotOrigin.getY() + (j * spacing);

                // Blue side
                Translation2d bluePos = new Translation2d(x, y);
                getGamePieceManager().spawnOnField(new RebuiltFuelOnField(bluePos));

                // Red side (mirrored)
                Translation2d redPos = flip(bluePos);
                getGamePieceManager().spawnOnField(new RebuiltFuelOnField(redPos));
            }
        }

        setupValueForMatchBreakdown("CurrentFuelInOutpost");
        setupValueForMatchBreakdown("TotalFuelInOutpost");
        setupValueForMatchBreakdown("TotalFuelInHub");
        setupValueForMatchBreakdown("WastedFuel");
    }

    @Override
    public synchronized List<Pose3d> getGamePiecesPosesByType(String type) {
        List<Pose3d> poses = super.getGamePiecesPosesByType(type);

        blueOutpost.draw(poses);
        redOutpost.draw(poses);

        return poses;
    }

    @Override
    public synchronized void clearGamePieces() {
        super.clearGamePieces();
        /* clear simulations */
        blueOutpost.clear();
        redOutpost.clear();
    }

    public void simulationSubTick(int tickNum) {
        if (shouldClock && !DriverStation.isAutonomous() && DriverStation.isEnabled()) {
            clock -= getSimulationDt().in(Units.Seconds);

            if (clock <= 0) {
                clock = 25;
                blueIsOnClock = !blueIsOnClock;
            }
        } else {
            clock = 25;
            String fmsMessage = DriverStation.getGameSpecificMessage();
            if (fmsMessage != null) {
                // 'B' means Blue is inactive first (Active in Shifts 2 & 4), so Red starts
                // Active (blueIsOnClock = false).
                if (fmsMessage.contains("B")) blueIsOnClock = false;
                // 'R' means Red is inactive first, so Blue starts Active (blueIsOnClock =
                // true).
                else if (fmsMessage.contains("R")) blueIsOnClock = true;
            }
        }

        phaseClockPublisher.set((clock));

        super.simulationSubTick(tickNum);

        blueActivePublisher.set(isActive(true));
        redActivePublisher.set(isActive(false));
    }

    /**
     *
     *
     * <h2>Returns wether the specified team currently has an active HUB</h2>
     *
     * This function returns true during autonomous or when shouldClock (set by {@link #setShouldRunClock(boolean)}) is
     * false.
     *
     * @param isBlue Wether to check the blue or red alliance.
     * @return Wether the specified alliance's HUB is currently active
     */
    public boolean isActive(boolean isBlue) {
        if (isBlue) {
            return blueIsOnClock || DriverStation.isAutonomous() || !shouldClock;
        } else {
            return !blueIsOnClock || DriverStation.isAutonomous() || !shouldClock;
        }
    }

    /**
     *
     *
     * <h2>Used to determine wether the arena should time which goal is active.</h2>
     *
     * When this is set too false both goals will always be set to active. Ths can be useful for testing or too simulate
     * endgame.
     *
     * @param shouldRunClock
     */
    public void setShouldRunClock(boolean shouldRunClock) {
        shouldClock = shouldRunClock;
    }

    /**
     *
     *
     * <h2>Dumps game pieces from the specified outpost.</h2>
     *
     * This function will dump up to 24 game pieces, dependent on how many game pieces are currently stored in the
     * outpost. For more manual control of the game pieces in the outpost use {@link #outpostThrow(boolean, Rotation2d,
     * Angle, LinearVelocity)}. To have a human player attempt to throw a game piece into the hub use
     * {@link #outpostThrowForGoal(boolean)}.
     *
     * @param isBlue wether to dump the blue or red outpost
     */
    public void outpostDump(boolean isBlue) {
        (isBlue ? blueOutpost : redOutpost).dump();
    }

    /**
     *
     *
     * <h2>Attempts too throw a game piece at the specified goal.</h2>
     *
     * <p>This method comes with variance built in (to simulate human inconsistency) and will therefore only hit about
     * half the time. Additionally if the hub does not have game pieces stored this method will not do anything. If you
     * would like to manually control how the human player throws game pieces use {@link #outpostThrow(boolean,
     * Rotation2d, Angle, LinearVelocity)}
     *
     * @param isBlue whether too throw for the blue or red HUB.
     */
    public void outpostThrowForGoal(boolean isBlue) {
        (isBlue ? blueOutpost : redOutpost).throwForGoal();
    }

    /**
     *
     *
     * <h2>Throws a game piece from the outpost at the specified angle and speed.</h2>
     *
     * <p>This method comes with variance built in (to simulate human inconsistency). Additionally if the hub does not
     * have game pieces stored this method will not do anything. If you would like to have the human player throw at the
     * hub use {@link #outpostThrowForGoal(boolean)}
     *
     * @param isBlue Wether too throw from the blue or red OUTPOST.
     * @param throwYaw The yaw at which too throw the ball.
     * @param throwPitch The pitch at which too throw the ball.
     * @param speed The speed at which too throw the ball.
     */
    public void outpostThrow(boolean isBlue, Rotation2d throwYaw, Angle throwPitch, LinearVelocity speed) {
        (isBlue ? blueOutpost : redOutpost).throwFuel(throwYaw, throwPitch, speed);
    }

    /**
     *
     *
     * <h2>Generates a random number within a range centered on 0 with a variance set by the parameter.</h2>
     *
     * @param variance the length of range used to generate the random number.
     * @return A random number in range.
     */
    public static double randomInRange(double variance) {
        return (Math.random() - 0.5) * variance;
    }
}
