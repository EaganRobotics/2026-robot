package org.ironmaple.simulation;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation.GamePieceInfo;
import org.ironmaple.simulation.gamepieces.GamePieceProjectile;
import org.ironmaple.simulation.gamepieces.GamePieceState;
import org.ironmaple.simulation.gamepieces.ManagedGamePiece;
import org.ironmaple.simulation.motorsims.MapleMotorSim;
import org.ironmaple.simulation.motorsims.SimMotorConfigs;

/**
 *
 *
 * <h2>Simulates a Shooter Mechanism.</h2>
 *
 * <p>Models flywheel physics, inputs from feed sources (like intakes), and launching of projectiles.
 */
public class ShooterSimulation implements SimulatedArena.Simulatable {
    private final GamePieceInfo gamePieceInfo;
    private final AbstractDriveTrainSimulation driveTrainSimulation;
    private final Supplier<Pose3d> shooterPoseSupplier;
    private final List<MapleMotorSim> flywheelMotorSims = new ArrayList<>();

    private IntakeSimulation intakeSource = null;
    private int gamePiecesLoaded = 0;
    private int capacity = 1;

    private Voltage appliedVoltage = Volts.zero();
    private Consumer<GamePieceProjectile> onShootCallback = (p) -> {};

    /**
     *
     *
     * <h2>Creates a new Shooter Simulation.</h2>
     *
     * @param gamePieceInfo the type of game piece this shooter handles
     * @param driveTrainSimulation the robot drive train (for calculating launch velocity from robot speed)
     * @param shooterPoseSupplier supplier for the shooter's pose <strong>relative to the robot center</strong>
     * @param flywheelMotors configurations for the flywheel motors
     */
    /**
     * Start building a Shooter Simulation.
     *
     * @param gamePieceInfo the type of game piece this shooter handles
     * @return a new Builder instance
     */
    public static Builder create(GamePieceInfo gamePieceInfo) {
        return new Builder(gamePieceInfo);
    }

    public static class Builder {
        private final GamePieceInfo gamePieceInfo;
        private AbstractDriveTrainSimulation driveTrainSimulation;
        private Supplier<Pose3d> shooterPoseSupplier;
        private int capacity = 1;
        private final List<SimMotorConfigs> motors = new ArrayList<>();
        private IntakeSimulation intakeSource = null;

        public Builder(GamePieceInfo gamePieceInfo) {
            this.gamePieceInfo = gamePieceInfo;
        }

        public Builder onRobot(AbstractDriveTrainSimulation drive, Supplier<Pose3d> poseSupplier) {
            this.driveTrainSimulation = drive;
            this.shooterPoseSupplier = poseSupplier;
            return this;
        }

        public Builder withMotor(SimMotorConfigs config) {
            this.motors.add(config);
            return this;
        }

        public Builder withCapacity(int capacity) {
            this.capacity = capacity;
            return this;
        }

        public Builder withIntakeSource(IntakeSimulation intake) {
            this.intakeSource = intake;
            return this;
        }

        public ShooterSimulation build() {
            if (driveTrainSimulation == null || shooterPoseSupplier == null) {
                throw new IllegalStateException("Drive simulation and shooter pose supplier must be set via onRobot()");
            }
            ShooterSimulation sim = new ShooterSimulation(
                    gamePieceInfo, driveTrainSimulation, shooterPoseSupplier, motors.toArray(new SimMotorConfigs[0]));
            sim.withCapacity(capacity);
            if (intakeSource != null) sim.withIntakeSource(intakeSource);
            return sim;
        }
    }

    private ShooterSimulation(
            GamePieceInfo gamePieceInfo,
            AbstractDriveTrainSimulation driveTrainSimulation,
            Supplier<Pose3d> shooterPoseSupplier,
            SimMotorConfigs... flywheelMotors) {
        this.gamePieceInfo = gamePieceInfo;
        this.driveTrainSimulation = driveTrainSimulation;
        this.shooterPoseSupplier = shooterPoseSupplier;

        for (SimMotorConfigs config : flywheelMotors) {
            MapleMotorSim sim = new MapleMotorSim(config);
            // Sets a dynamic controller that reads the current applied voltage
            sim.useMotorController((mechAngle, mechVel, encAngle, encVel) -> appliedVoltage);
            this.flywheelMotorSims.add(sim);
        }

        // Automatically register with the arena
        SimulatedArena.getInstance().addCustomSimulation(this);
    }

    /**
     *
     *
     * <h2>Pairs this shooter with an Intake.</h2>
     *
     * <p>The shooter will automatically poll the intake for game pieces during the simulation loop. If the intake has a
     * game piece and the shooter has space, it will be transferred.
     *
     * @param intake the intake simulation to pull from
     * @return this instance
     */
    public ShooterSimulation withIntakeSource(IntakeSimulation intake) {
        this.intakeSource = intake;
        return this;
    }

    /**
     *
     *
     * <h2>Sets the storage capacity (ammo limit).</h2>
     *
     * @param capacity max game pieces
     * @return this instance
     */
    public ShooterSimulation withCapacity(int capacity) {
        this.capacity = capacity;
        return this;
    }

    /**
     *
     *
     * <h2>Sets the voltage for the flywheel motors.</h2>
     *
     * @param voltage target voltage
     */
    public void setShooterVoltage(Voltage voltage) {
        this.appliedVoltage = voltage;
    }

    /**
     *
     *
     * <h2>Manually adds game pieces to the shooter.</h2>
     *
     * @param amount number to add
     */
    public void addGamePiece(int amount) {
        this.gamePiecesLoaded = Math.min(this.gamePiecesLoaded + amount, capacity);
    }

    /**
     *
     *
     * <h2>Checks if a game piece is loaded.</h2>
     *
     * @return true if count > 0
     */
    public boolean hasGamePiece() {
        return gamePiecesLoaded > 0;
    }

    public int getGamePiecesLoaded() {
        return gamePiecesLoaded;
    }

    /**
     *
     *
     * <h2>Gets the average angular velocity of the flywheels.</h2>
     *
     * @return average velocity
     */
    public AngularVelocity getVelocity() {
        if (flywheelMotorSims.isEmpty()) return RadiansPerSecond.zero();
        AngularVelocity total = RadiansPerSecond.zero();
        for (MapleMotorSim sim : flywheelMotorSims) {
            total = total.plus(sim.getVelocity());
        }
        return total.div(flywheelMotorSims.size());
    }

    public Current getStatorCurrent() {
        Current total = Amps.zero();
        for (MapleMotorSim sim : flywheelMotorSims) {
            total = total.plus(sim.getStatorCurrent());
        }
        return total;
    }

    /**
     *
     *
     * <h2>Shoots a game piece.</h2>
     *
     * <p>Creates a projectile if ammo is available.
     *
     * @param velocity the muzzle velocity of the game piece
     * @param pitch the launch angle (up/down) relative to the shooter's horizontal
     */
    public void shoot(LinearVelocity velocity, Angle pitch) {
        if (gamePiecesLoaded <= 0) return;

        gamePiecesLoaded--;

        // 1. Calculate Shooter Pose in World Frame
        Pose3d robotPose = new Pose3d(driveTrainSimulation.getSimulatedDriveTrainPose());
        Pose3d shooterPoseRelative = shooterPoseSupplier.get();
        // Transform3d from robot center to shooter
        Transform3d robotToShooter =
                new Transform3d(shooterPoseRelative.getTranslation(), shooterPoseRelative.getRotation());
        Pose3d shooterPoseWorld = robotPose.plus(robotToShooter);

        // 2. Calculate Muzzle Velocity in World Frame
        // 2a. Muzzle velocity relative to shooter
        double vMuzzle = velocity.in(edu.wpi.first.units.Units.MetersPerSecond);
        double pitchRad = pitch.in(Radians);

        // Assuming pitch is rotation around local Y axis (up/down), and shooter faces
        // local X
        double vXLocal = vMuzzle * Math.cos(pitchRad);
        double vZLocal = vMuzzle * Math.sin(pitchRad);

        // 2b. Rotate vXLocal by shooter's yaw (global) to get vXYWorld
        Rotation2d shooterYaw = shooterPoseWorld.getRotation().toRotation2d();
        Translation2d vXYWorldFromMuzzle = new Translation2d(vXLocal, 0).rotateBy(shooterYaw);

        // 2c. Add Robot Velocity
        ChassisSpeeds robotSpeeds = driveTrainSimulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
        Translation2d robotVXY = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
        Translation2d totalVXY = vXYWorldFromMuzzle.plus(robotVXY);

        // 3. Create Projectile
        GamePieceProjectile projectile = new GamePieceProjectile(
                gamePieceInfo,
                shooterPoseWorld.getTranslation().toTranslation2d(),
                totalVXY,
                shooterPoseWorld.getZ(),
                vZLocal, // Ignoring vertical robot velocity for simplicity unless we want to add it
                shooterPoseWorld.getRotation().plus(new Rotation3d(0, -pitchRad, 0)) // Visual rotation
                );

        SimulatedArena.getInstance().getGamePieceManager().spawnInFlight(projectile);
        projectile.launch();
        onShootCallback.accept(projectile);
    }

    /**
     *
     *
     * <h2>Registers a callback for when a shot is fired.</h2>
     *
     * @param callback consumer that receives the fired projectile
     * @return this instance
     */
    public ShooterSimulation onShoot(Consumer<GamePieceProjectile> callback) {
        this.onShootCallback = callback;
        return this;
    }

    /**
     *
     *
     * <h2>Supplies poses for game pieces currently loaded in this shooter.</h2>
     *
     * <p>Use this method to visualize pieces inside the robot in AdvantageScope.
     *
     * @param type the game piece type to query
     * @param poseList the list to add poses to
     */
    public void supplyRobotPoses(String type, List<Pose3d> poseList) {
        if (!type.equals(gamePieceInfo.type())) return;
        Pose3d robotPose = new Pose3d(driveTrainSimulation.getSimulatedDriveTrainPose());
        Pose3d shooterPoseRelative = shooterPoseSupplier.get();
        for (int i = 0; i < gamePiecesLoaded; i++) {
            Transform3d offset = new Transform3d(
                    shooterPoseRelative.getTranslation().plus(new Translation3d(0, 0, i * 0.05)),
                    shooterPoseRelative.getRotation());
            poseList.add(robotPose.plus(offset));
        }
    }

    @Override
    public void simulationSubTick(int subTickNum) {
        // Update motors
        for (MapleMotorSim sim : flywheelMotorSims) {
            sim.update(SimulatedArena.getSimulationDt());
        }

        // Poll intake
        if (intakeSource != null && gamePiecesLoaded < capacity) {
            // Transfer 1 piece per tick if available.
            if (intakeSource.obtainGamePieceFromIntake()) {
                gamePiecesLoaded++;

                // Transition a piece from IN_INTAKE to IN_SHOOTER in manager
                List<ManagedGamePiece> intakePieces =
                        SimulatedArena.getInstance().getGamePieceManager().getByOwner(intakeSource);
                if (!intakePieces.isEmpty()) {
                    ManagedGamePiece piece = intakePieces.get(0);
                    piece.setState(GamePieceState.IN_SHOOTER);
                    piece.setOwner(this);
                }
            }
        }
    }
}
