package frc.robot26.subsystems.shooter.setpoint;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Distance;

public class ShooterDistanceTable {
  private static final ShooterInterpolationMap table = new ShooterInterpolationMap();

  static {
    table.put(5, new ShooterSetpoint(RPM.of(450), RPM.of(3000), Degree.of(0)));
    table.put(7.5, new ShooterSetpoint(RPM.of(510), RPM.of(3000), Degree.of(5)));
    table.put(9, new ShooterSetpoint(RPM.of(515), RPM.of(3000), Degree.of(7.75)));
    table.put(11.5, new ShooterSetpoint(RPM.of(520), RPM.of(3000), Degree.of(17.5)));
    table.put(13, new ShooterSetpoint(RPM.of(530), RPM.of(3000), Degree.of(18)));
    table.put(15, new ShooterSetpoint(RPM.of(550), RPM.of(3000), Degree.of(20)));
  }

  public static ShooterSetpoint getShooterSetpoint(Distance distance) {
    return table.get(distance.in(Feet));
  }
}
