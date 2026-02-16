package frc.robot26.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.tunables.LoggedTunablePIDs;

public class ShooterConstants {
  public static final double joystickSpeedMultiplier = 0.314;
  public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(25);
  public static final double GEARING_SHOOTER = 5.0; // TODO: adjust
  public static final double GEARING_HOOD = 5.0; // TODO: adjust

  public static final Angle hoodRotationStartLimit = Rotations.of(0.0); // TODO: adjust
  public static final Angle hoodRotationEndLimit = Rotations.of(5.67); // TODO: adjust

  public static final class Real {
    public static final int followerLeftMotorID = 26; // TODO: change to correct ID
    public static final int leadLeftMotorID = 27; // TODO: change to correct ID
    public static final int followerRightMotorID = 28; // TODO: change to correct ID
    public static final int leadRightMotorID = 29; // TODO: change to correct ID
    public static final int hoodMotorID = 30; // TODO: change to correct ID

    public static final LoggedTunablePIDs shooterPIDs =
        new LoggedTunablePIDs("Shooter", 1.0, 0.1, 0.1);
    public static final LoggedTunablePIDs hoodPIDs = new LoggedTunablePIDs("Hood", 1.0, 0.1, 0.1);
    // Separate, tunable PID set specifically for the shooter velocity closed-loop on the Talon
    public static final LoggedTunablePIDs shooterVelocityPIDs =
        new LoggedTunablePIDs("Shooter/Velocity", 1.0, 0.1, 0.1);
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
