package frc.robot26.subsystems.shooter.setpoint;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.units.measure.Distance;

public class ShooterDistanceTable {
  private static final ShooterInterpolationMap table = new ShooterInterpolationMap();

  static {
    table.put(5.0, new ShooterSetpoint(RPM.of(2400), RPM.of(3000), Degree.of(0)));
    table.put(6.0, new ShooterSetpoint(RPM.of(2500), RPM.of(3000), Degree.of(9)));
    table.put(8.0, new ShooterSetpoint(RPM.of(2600), RPM.of(3000), Degree.of(11)));
    table.put(9, new ShooterSetpoint(RPM.of(2650), RPM.of(3000), Degree.of(12.5)));
    table.put(10, new ShooterSetpoint(RPM.of(2700), RPM.of(3000), Degree.of(13)));
    table.put(11, new ShooterSetpoint(RPM.of(2700), RPM.of(3000), Degree.of(20)));
    table.put(12, new ShooterSetpoint(RPM.of(2700), RPM.of(3000), Degree.of(23)));
    table.put(13, new ShooterSetpoint(RPM.of(2750), RPM.of(3000), Degree.of(25)));
    table.put(15, new ShooterSetpoint(RPM.of(2850), RPM.of(3000), Degree.of(27)));
    table.put(18, new ShooterSetpoint(RPM.of(3200), RPM.of(3000), Degree.of(27)));

    // 5 ft options
    // option 1: table.put(5, new ShooterSetpoint(RPM.of(2435), RPM.of(3000), Degree.of(0)));
    // option 2: table.put(5, new ShooterSetpoint(RPM.of(2451), RPM.of(3000), Degree.of(0.25)));
    // option 3: table.put(5, new ShooterSetpoint(RPM.of(2468), RPM.of(3000), Degree.of(0.5)));
    // 6.44 ft options
    // option 1: table.put(6.44, new ShooterSetpoint(RPM.of(2600), RPM.of(3000), Degree.of(9.25)));
    // option 2: table.put(6.44, new ShooterSetpoint(RPM.of(2569), RPM.of(3000), Degree.of(8.75)));
    // option 3: table.put(6.44, new ShooterSetpoint(RPM.of(2539), RPM.of(3000), Degree.of(8.25)));
    // 7.89 ft options
    // option 1: table.put(7.89, new ShooterSetpoint(RPM.of(2481), RPM.of(3000), Degree.of(11)));
    // option 2: table.put(7.89, new ShooterSetpoint(RPM.of(2471), RPM.of(3000), Degree.of(10.75)));
    // option 3: table.put(7.89, new ShooterSetpoint(RPM.of(2491), RPM.of(3000), Degree.of(11.25)));
    // 9.33 ft options
    // option 1: table.put(9.33, new ShooterSetpoint(RPM.of(2480), RPM.of(3000), Degree.of(12.75)));
    // option 2: table.put(9.33, new ShooterSetpoint(RPM.of(2471), RPM.of(3000), Degree.of(12.5)));
    // option 3: table.put(9.33, new ShooterSetpoint(RPM.of(2488), RPM.of(3000), Degree.of(13)));
    // 10.78 ft options
    // option 1: table.put(10.78, new ShooterSetpoint(RPM.of(2657), RPM.of(3000), Degree.of(18.5)));
    // option 2: table.put(10.78, new ShooterSetpoint(RPM.of(2667), RPM.of(3000),
    // Degree.of(18.75)));
    // option 3: table.put(10.78, new ShooterSetpoint(RPM.of(2648), RPM.of(3000),
    // Degree.of(18.25)));
    // 12.22 ft options
    // option 1: table.put(12.22, new ShooterSetpoint(RPM.of(2832), RPM.of(3000),
    // Degree.of(23.25)));
    // option 2: table.put(12.22, new ShooterSetpoint(RPM.of(2843), RPM.of(3000), Degree.of(23.5)));
    // option 3: table.put(12.22, new ShooterSetpoint(RPM.of(2822), RPM.of(3000), Degree.of(23)));
    // 13.67 ft options
    // option 1: table.put(13.67, new ShooterSetpoint(RPM.of(2925), RPM.of(3000), Degree.of(25.5)));
    // option 2: table.put(13.67, new ShooterSetpoint(RPM.of(2914), RPM.of(3000),
    // Degree.of(25.25)));
    // option 3: table.put(13.67, new ShooterSetpoint(RPM.of(2936), RPM.of(3000),
    // Degree.of(25.75)));
    // 15.11 ft options
    // option 1: table.put(15.11, new ShooterSetpoint(RPM.of(3000), RPM.of(3000), Degree.of(27)));
    // option 2: table.put(15.11, new ShooterSetpoint(RPM.of(2989), RPM.of(3000),
    // Degree.of(26.75)));
    // option 3: table.put(15.11, new ShooterSetpoint(RPM.of(2979), RPM.of(3000), Degree.of(26.5)));
    // 16.56 ft options
    // option 1: table.put(16.56, new ShooterSetpoint(RPM.of(3025), RPM.of(3000), Degree.of(27)));
    // option 2: table.put(16.56, new ShooterSetpoint(RPM.of(3035), RPM.of(3000),
    // Degree.of(27.25)));
    // option 3: table.put(16.56, new ShooterSetpoint(RPM.of(3015), RPM.of(3000),
    // Degree.of(26.75)));
    // 18 ft options
    // option 1: table.put(18, new ShooterSetpoint(RPM.of(3062), RPM.of(3000), Degree.of(27)));
    // option 2: table.put(18, new ShooterSetpoint(RPM.of(3071), RPM.of(3000), Degree.of(27.25)));
    // option 3: table.put(18, new ShooterSetpoint(RPM.of(3053), RPM.of(3000), Degree.of(26.75)));

  }

  public static ShooterSetpoint getShooterSetpoint(Distance distance) {
    return table.get(distance.in(Feet));
  }
}
