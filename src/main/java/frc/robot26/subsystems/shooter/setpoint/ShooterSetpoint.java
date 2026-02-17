package frc.robot26.subsystems.shooter.setpoint;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/** Data class that holds shooter setpoint values: shooter speed, feeder speed, and hood angle. */
public final class ShooterSetpoint {
  public final AngularVelocity shooterSpeed;
  public final AngularVelocity feederSpeed;
  public final Angle hoodAngle;

  /**
   * Create a shooter setpoint with all three values.
   *
   * @param shooterSpeed the shooter angular velocity
   * @param feederSpeed the feeder angular velocity
   * @param hoodAngle the hood angle
   */
  public ShooterSetpoint(
      AngularVelocity shooterSpeed, AngularVelocity feederSpeed, Angle hoodAngle) {
    this.shooterSpeed = shooterSpeed;
    this.feederSpeed = feederSpeed;
    this.hoodAngle = hoodAngle;
  }
}
