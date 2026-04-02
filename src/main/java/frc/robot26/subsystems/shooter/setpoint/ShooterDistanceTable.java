package frc.robot26.subsystems.shooter.setpoint;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Distance;

public class ShooterDistanceTable {
  private static final ShooterInterpolationMap table = new ShooterInterpolationMap();

  static {
    table.put(7.5, new ShooterSetpoint(RPM.of(535), RPM.of(4000), Degree.of(0)));
    table.put(11.5, new ShooterSetpoint(RPM.of(510), RPM.of(4000), Degree.of(20)));
    table.put(6, new ShooterSetpoint(RPM.of(445), RPM.of(4000), Degree.of(0)));
    // table.put(7, new ShooterSetpoint(RPM.of(620), RPM.of(3000), Degree.of(0)));
    // table.put(8, new ShooterSetpoint(RPM.of(660), RPM.of(4000), Degree.of(0)));
    // table.put(9, new ShooterSetpoint(RPM.of(700), RPM.of(4000), Degree.of(0)));
    // table.put(10, new ShooterSetpoint(RPM.of(650), RPM.of(4000), Radians.of(0.795)));
    // table.put(11, new ShooterSetpoint(RPM.of(650), RPM.of(4000), Radians.of(1.5)));
    // table.put(12, new ShooterSetpoint(RPM.of(650), RPM.of(4000), Radians.of(1.5)));
    // table.put(13, new ShooterSetpoint(RPM.of(620), RPM.of(4000), Radians.of(3.5)));
  }

  public static ShooterSetpoint getShooterSetpoint(Distance distance) {
    return table.get(distance.in(Feet));
  }
}
