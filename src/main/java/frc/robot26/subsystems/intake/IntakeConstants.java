package frc.robot26.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.tunables.LoggedTunablePIDs;

public class IntakeConstants {
  public static final double joystickSpeedMultiplier = 0.85; // TODO: change
  public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(5); // TODO: change
  public static final double GEARING_INTAKE = 1.25;
  public static final double GEARING_DEPLOY = 10.0; // TODO: adjust (10:1 is an estimate)

  public static final Angle deployRotationLimit = Rotations.of(5.67); // TODO: adjust
  public static final Angle retractRotationLimit = Rotations.of(0); // TODO: adjust

  public static final class Real {
    public static final int followerMotorID = 25; // TODO: change to correct ID
    public static final int leadMotorID = 17;
    public static final int deployMotorID = 18;
    public static final int limitSwitchChannel = 0;

    public static final LoggedTunablePIDs deployPIDs =
        new LoggedTunablePIDs("Deploy", 1.0, 0.1, 0.1); // TODO: change
  }

  public static final class Sim {
    public static final double kP = 4.0; // 5
    public static final double kI = 0.3;
    public static final double kD = 0.6;
    public static final double kS = 0.0;
    public static final double kG = 0.43; // 0.37
    public static final double kV = 0.10146; // 2.67
    public static final double kA = 0.002; // * DRUM_RADIUS.in(Meters); // 0.05
    public static final MomentOfInertia MOTOR_LOAD_MOI = KilogramSquareMeters.of(0.04); // TODO
    // estimate
    public static final Voltage FRICTION_VOLTAGE = Volts.of(0.5);
  }
}
