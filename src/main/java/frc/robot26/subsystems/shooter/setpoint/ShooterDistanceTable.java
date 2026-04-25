package frc.robot26.subsystems.shooter.setpoint;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Distance;

public class ShooterDistanceTable {
  private static final ShooterInterpolationMap table = new ShooterInterpolationMap();

  static {
    table.put(5, new ShooterSetpoint(RPM.of(450 * 5), RPM.of(3000), Degree.of(0)));
    table.put(7.5, new ShooterSetpoint(RPM.of(510 * 5), RPM.of(3000), Degree.of(5)));
    table.put(9, new ShooterSetpoint(RPM.of(515 * 5), RPM.of(3000), Degree.of(7.75)));
    table.put(11.5, new ShooterSetpoint(RPM.of(520 * 5), RPM.of(3000), Degree.of(17.5)));
    table.put(13, new ShooterSetpoint(RPM.of(530 * 5), RPM.of(3000), Degree.of(18)));
    table.put(15, new ShooterSetpoint(RPM.of(575 * 5), RPM.of(3000), Degree.of(20)));
    table.put(16.5, new ShooterSetpoint(RPM.of(600 * 5), RPM.of(3000), Degree.of(20)));
    table.put(18, new ShooterSetpoint(RPM.of(620 * 5), RPM.of(3000), Degree.of(20)));
    table.put(19.5, new ShooterSetpoint(RPM.of(627 * 5), RPM.of(3000), Degree.of(20)));
  }

  public static ShooterSetpoint getShooterSetpoint(Distance distance) {
    return table.get(distance.in(Feet));
  }
}
