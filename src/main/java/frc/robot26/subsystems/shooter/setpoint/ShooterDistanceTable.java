package frc.robot26.subsystems.shooter.setpoint;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Distance;

public class ShooterDistanceTable {
  private static final ShooterInterpolationMap table = new ShooterInterpolationMap();

  static {
    table.put(5.0, new ShooterSetpoint(RPM.of(2400), RPM.of(3000), Degree.of(0)));
    // table.put(5.5, new ShooterSetpoint(RPM.of(2500), RPM.of(5000), Degree.of(2.5)));
    table.put(6.0, new ShooterSetpoint(RPM.of(2500), RPM.of(3000), Degree.of(9)));
    // table.put(7.5, new ShooterSetpoint(RPM.of(510), RPM.of(5000), Degree.of(20)));
    table.put(8.0, new ShooterSetpoint(RPM.of(2600), RPM.of(3000), Degree.of(11)));
    // table.put(5, new ShooterSetpoint(RPM.of(450 * 5), RPM.of(3000), Degree.of(0)));
    // table.put(7.5, new ShooterSetpoint(RPM.of(510 * 5), RPM.of(3000), Degree.of(5)));
    table.put(9, new ShooterSetpoint(RPM.of(2650), RPM.of(3000), Degree.of(12.5)));
    table.put(10, new ShooterSetpoint(RPM.of(2700), RPM.of(3000), Degree.of(13)));
    table.put(11, new ShooterSetpoint(RPM.of(2700), RPM.of(3000), Degree.of(20)));
    table.put(12, new ShooterSetpoint(RPM.of(2700), RPM.of(3000), Degree.of(23)));
    table.put(13, new ShooterSetpoint(RPM.of(2750), RPM.of(3000), Degree.of(25)));
    table.put(15, new ShooterSetpoint(RPM.of(2850), RPM.of(3000), Degree.of(27)));
    table.put(18, new ShooterSetpoint(RPM.of(3200), RPM.of(3000), Degree.of(27)));
    // table.put(11.5, new ShooterSetpoint(RPM.of(520 * 5), RPM.of(3000), Degree.of(17.5)));
    // table.put(13, new ShooterSetpoint(RPM.of(530 * 5), RPM.of(3000), Degree.of(18)));
    // table.put(15, new ShooterSetpoint(RPM.of(575 * 5), RPM.of(3000), Degree.of(20)));
    // table.put(16.5, new ShooterSetpoint(RPM.of(600 * 5), RPM.of(3000), Degree.of(20)));
    // table.put(18, new ShooterSetpoint(RPM.of(620 * 5), RPM.of(3000), Degree.of(20)));
    // table.put(19.5, new ShooterSetpoint(RPM.of(627 * 5), RPM.of(3000), Degree.of(20)));
  }

  public static ShooterSetpoint getShooterSetpoint(Distance distance) {
    return table.get(distance.in(Feet));
  }
}
