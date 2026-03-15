package org.ironmaple.simulation.seasonspecific.rebuilt2026;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Pounds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.dyn4j.geometry.Circle;
import org.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;

public class RebuiltFuelOnField extends GamePieceOnFieldSimulation {

    public static final GamePieceInfo REBUILT_FUEL_INFO =
            new GamePieceInfo("Fuel", new Circle(0.075), Centimeters.of(15.0), Pounds.of(0.474), 1.8, 5, 0.8);

    public RebuiltFuelOnField(Translation2d initialPosition) {
        super(REBUILT_FUEL_INFO, new Pose2d(initialPosition, Rotation2d.kZero));
    }
}
