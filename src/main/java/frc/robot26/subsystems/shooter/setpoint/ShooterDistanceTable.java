package frc.robot26.subsystems.shooter.setpoint;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Distance;

public class ShooterDistanceTable {
  private static final ShooterInterpolationMap table = new ShooterInterpolationMap();

  static {
    table.put(1, new ShooterSetpoint(RPM.of(4000), RPM.of(4000), Degree.of(30)));
    table.put(2, new ShooterSetpoint(RPM.of(4000), RPM.of(4000), Degree.of(30)));
    table.put(3, new ShooterSetpoint(RPM.of(4000), RPM.of(4000), Degree.of(30)));
    table.put(4, new ShooterSetpoint(RPM.of(4000), RPM.of(4000), Degree.of(30)));
    table.put(5, new ShooterSetpoint(RPM.of(4000), RPM.of(4000), Degree.of(30)));
    table.put(6, new ShooterSetpoint(RPM.of(4000), RPM.of(4000), Degree.of(30)));
    table.put(7, new ShooterSetpoint(RPM.of(4000), RPM.of(4000), Degree.of(30)));
    table.put(8, new ShooterSetpoint(RPM.of(4000), RPM.of(4000), Degree.of(30)));
    table.put(9, new ShooterSetpoint(RPM.of(4000), RPM.of(4000), Degree.of(30)));
  }

  public static ShooterSetpoint getShooterSetpoint(Distance distance) {
    return table.get(distance.in(Feet));
  }
}
