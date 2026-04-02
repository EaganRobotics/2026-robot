package frc.lib.tunables;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class LoggedTunablePIDs {
  private final LoggedTunableNumber kP;
  private final LoggedTunableNumber kI;
  private final LoggedTunableNumber kD;

  public LoggedTunablePIDs(String name, double defaultKP, double defaultKI, double defaultKD) {
    this.kP = new LoggedTunableNumber("Tuning/" + name + "/kP", defaultKP);
    this.kI = new LoggedTunableNumber("Tuning/" + name + "/kI", defaultKI);
    this.kD = new LoggedTunableNumber("Tuning/" + name + "/kD", defaultKD);
  }

  public void applyToTalonFXConfig(TalonFX talon, TalonFXConfiguration config) {
    kP.addListener(
        kP -> {
          config.Slot0.kP = kP;
          talon.getConfigurator().apply(config);
        });
    kI.addListener(
        kI -> {
          config.Slot0.kI = kI;
          talon.getConfigurator().apply(config);
        });
    kD.addListener(
        kD -> {
          config.Slot0.kD = kD;
          talon.getConfigurator().apply(config);
        });
  }

  public ProfiledPIDController createController(double maxVelocity, double maxAcceleration) {
    ProfiledPIDController controller =
        new ProfiledPIDController(
            kP.getValue(),
            kI.getValue(),
            kD.getValue(),
            new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));

    kP.addListener(value -> controller.setP(value));
    kI.addListener(value -> controller.setI(value));
    kD.addListener(value -> controller.setD(value));

    return controller;
  }

  public double getKP() {
    return kP.getValue();
  }

  public double getKI() {
    return kI.getValue();
  }

  public double getKD() {
    return kD.getValue();
  }
}
