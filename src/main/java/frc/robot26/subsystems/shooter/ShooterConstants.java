package frc.robot26.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.tunables.*;

public class ShooterConstants {
  public static final double joystickSpeedMultiplier = 0.85;
  public static final Current SUPPLY_CURRENT_LIMIT = Amps.of(25);
  public static final double GEARING_SHOOTER = 5.0;
  public static final double GEARING_HOOD = 1.0;

  public static final Angle hoodRotationStartLimit = Radians.of(0.0);
  public static final Angle hoodRotationEndLimit = Radians.of(9.68);

  public static final class Real {
    public static final int followerLeftMotorID = 13;
    public static final int leadLeftMotorID = 14;
    public static final int followerRightMotorID = 11;
    public static final int leadRightMotorID = 12;
    public static final int hoodMotorID = 16;

    public static final LoggedTunablePIDs shooterPIDs =
        new LoggedTunablePIDs("Shooter", 2.0, 0.0, 0.0);
    public static final LoggedTunablePIDs hoodPIDs = new LoggedTunablePIDs("Hood", 0.01, 0.0, 0.0);
    public static final LoggedTunableNumber shooterSpeed =
        new LoggedTunableNumber("Tuning/ShooterSpeed", 500);
    public static final LoggedTunableNumber hoodAngle =
        new LoggedTunableNumber("Tuning/HoodAngle", 550); // TODO: change
    public static final LoggedTunableNumber hoodAngleBack =
        new LoggedTunableNumber("Tuning/hoodAngleBack", 0); // TODO: change
    public static final LoggedTunableNumber shooterAcceleration =
        new LoggedTunableNumber("Tuning/ShooterAcceleration", 500);
    public static final LoggedTunableNumber hoodCruiseVelocity =
        new LoggedTunableNumber("Tuning/hoodCruiseVelocity", 1000);
    public static final LoggedTunableNumber hoodAcceleration =
        new LoggedTunableNumber("Tuning/hoodAcceleration", 1000);
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
        KilogramSquareMeters.of(0.04); // TODO estimatee
    public static final Voltage FRICTION_VOLTAGE = Volts.of(0.5);
  }
}
