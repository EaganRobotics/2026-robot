package frc.robot26.subsystems.shooter.setpoint;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Interpolating map that uses three InterpolatingDoubleTreeMap instances (one for each parameter:
 * shooterSpeed, feederSpeed, hoodAngle). Allows putting and getting ShooterSetpoint objects keyed
 * by distance.
 */
public final class ShooterInterpolationMap {
  private final InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap feederSpeedMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

  /**
   * Put a shooter setpoint at a specific distance.
   *
   * @param distance the distance in meters
   * @param setpoint the shooter setpoint containing all three values
   */
  public void put(double distance, ShooterSetpoint setpoint) {
    shooterSpeedMap.put(distance, setpoint.shooterSpeed.in(RPM));
    feederSpeedMap.put(distance, setpoint.feederSpeed.in(RPM));
    hoodAngleMap.put(distance, setpoint.hoodAngle.in(Rotations));
  }

  /**
   * Get an interpolated shooter setpoint at a specific distance.
   *
   * @param distance the distance in meters
   * @return the interpolated ShooterSetpoint containing shooterSpeed, feederSpeed, and hoodAngle
   */
  public ShooterSetpoint get(double distance) {
    AngularVelocity shooterSpeed = RPM.of(shooterSpeedMap.get(distance));
    AngularVelocity feederSpeed = RPM.of(feederSpeedMap.get(distance));
    Angle hoodAngle = Rotations.of(hoodAngleMap.get(distance));

    return new ShooterSetpoint(shooterSpeed, feederSpeed, hoodAngle);
  }
}
