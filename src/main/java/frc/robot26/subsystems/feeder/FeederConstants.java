package frc.robot26.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.tunables.*;

public class FeederConstants {
  public static final double joystickSpeedMultiplier =
      Math.E + 0.7; // there is no reason this is like this i just thought it was funny
  public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(25);
  public static final double GEARING = 1.5;

  public static final class Real {
    public static final int followerMotorID = 10;
    public static final int leadMotorID = 9;

    public static final LoggedTunablePIDs feederPIDs =
        new LoggedTunablePIDs("Feeder", 0.5, 0.0, 0.001);
    public static final LoggedTunableNumber feederSpeed =
        new LoggedTunableNumber("Tuning/FeederSpeed", 2000);
  }

  public static final class Sim {
    public static final double kP = 4.0; // 5
    public static final double kI = 0.3;
    public static final double kD = 0.6;
    public static final double kS = 0.0;
    public static final double kG = 0.43; // 0.37
    public static final double kV = 0.10146; // 2.67
    public static final double kA = 0.002; // * DRUM_RADIUS.in(Meters); // 0.05
    public static final MomentOfInertia MOTOR_LOAD_MOI = KilogramSquareMeters.of(0.04);
    public static final Voltage FRICTION_VOLTAGE = Volts.of(0.5);
  }
}
