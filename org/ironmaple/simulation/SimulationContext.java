package org.ironmaple.simulation;

import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

/**
 * A context record that groups the primary simulation components (Drive and Arena).
 *
 * <p>This is designed to be passed around to subsystem simulations (IO layers) to avoid static global access
 * (dependency injection pattern).
 */
public record SimulationContext(AbstractDriveTrainSimulation driveTrainSimulation, SimulatedArena simulatedArena) {
    /**
     * Creates a new context with the given drive simulation and the singleton arena instance.
     *
     * @param driveTrainSimulation the drive train simulation
     * @return a new SimulationContext
     */
    public static SimulationContext of(AbstractDriveTrainSimulation driveTrainSimulation) {
        return new SimulationContext(driveTrainSimulation, SimulatedArena.getInstance());
    }
}
