package org.ironmaple.simulation.gamepieces;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.UUID;

/**
 *
 *
 * <h2>Wrapper for a managed game piece with state tracking.</h2>
 *
 * <p>Tracks the lifecycle state, owner, and pose of a game piece in the simulation.
 */
public class ManagedGamePiece {
    private final UUID id;
    private final String type;
    private GamePieceState state;
    private Object owner; // IntakeSimulation, ShooterSimulation, Goal, or null
    private GamePiece underlying; // The physics object (if on field or in flight)

    /**
     * Creates a new managed game piece.
     *
     * @param type the game piece type (e.g., "Fuel", "Note")
     * @param state initial state
     * @param underlying the underlying physics object
     */
    public ManagedGamePiece(String type, GamePieceState state, GamePiece underlying) {
        this.id = UUID.randomUUID();
        this.type = type;
        this.state = state;
        this.underlying = underlying;
        this.owner = null;
    }

    public UUID getId() {
        return id;
    }

    public String getType() {
        return type;
    }

    public GamePieceState getState() {
        return state;
    }

    public void setState(GamePieceState state) {
        this.state = state;
    }

    public Object getOwner() {
        return owner;
    }

    public void setOwner(Object owner) {
        this.owner = owner;
    }

    public GamePiece getUnderlying() {
        return underlying;
    }

    public void setUnderlying(GamePiece underlying) {
        this.underlying = underlying;
    }

    /**
     * Gets the current pose of this game piece.
     *
     * @return the 3D pose, or null if not available
     */
    public Pose3d getPose() {
        if (underlying != null) {
            return underlying.getPose3d();
        }
        return null;
    }

    /**
     * Checks if this piece is currently on the field (physics-backed).
     *
     * @return true if ON_FIELD state
     */
    public boolean isOnField() {
        return state == GamePieceState.ON_FIELD;
    }

    /**
     * Checks if this piece is in flight.
     *
     * @return true if IN_FLIGHT state
     */
    public boolean isInFlight() {
        return state == GamePieceState.IN_FLIGHT;
    }

    /**
     * Checks if this piece is held by a robot mechanism.
     *
     * @return true if IN_INTAKE or IN_SHOOTER
     */
    public boolean isInRobot() {
        return state == GamePieceState.IN_INTAKE || state == GamePieceState.IN_SHOOTER;
    }
}
