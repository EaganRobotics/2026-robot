package org.ironmaple.simulation.gamepieces;

/**
 *
 *
 * <h2>Represents the state of a game piece in the simulation.</h2>
 */
public enum GamePieceState {
    /** Game piece is on the field (physics-simulated). */
    ON_FIELD,
    /** Game piece is held in an intake mechanism. */
    IN_INTAKE,
    /** Game piece is loaded in a shooter mechanism. */
    IN_SHOOTER,
    /** Game piece is in flight (projectile). */
    IN_FLIGHT,
    /** Game piece has been scored in a goal. */
    IN_GOAL
}
