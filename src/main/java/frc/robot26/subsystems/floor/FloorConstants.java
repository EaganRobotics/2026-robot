package frc.robot26.subsystems.floor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.tunables.LoggedTunableNumber;
import frc.lib.tunables.LoggedTunablePIDs;

public class FloorConstants {
  public static final double joystickSpeedMultiplier = 0.314; // TODO: change
  public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(50); // TODO: change
  public static final double GEARING = 3.0;

  public static final LoggedTunablePIDs floorPIDs =
      new LoggedTunablePIDs("Floor", 0.5, 0.0, 0.0); // TODO: change
  public static final LoggedTunableNumber floorSpeed =
      new LoggedTunableNumber("Tuning/FloorSpeed", 6000);

  public static final class Real {
    public static final int leadMotorID = 15;
  }

  public static final class Sim {
    public static final double kP = 4.0; // 5
    public static final double kI = 0.3;
    public static final double kD = 0.6;
    public static final double kS = 0.0;
    public static final double kG = 0.43; // 0.37
    public static final double kV = 0.10146; // 2.67
    public static final double kA = 0.002; // * DRUM_RADIUS.in(Meters); // 0.05
    public static final MomentOfInertia MOTOR_LOAD_MOI =
        KilogramSquareMeters.of(0.04); // TODO estimate
    public static final Voltage FRICTION_VOLTAGE = Volts.of(0.5);
  }
}
