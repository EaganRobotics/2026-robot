package frc.robot26.subsystems.drive;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot26.generated.TunerConstants;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

public class DriveSimConfig {
  // Maple Sim config constants
  public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG =
      DriveTrainSimulationConfig.Default()
          .withRobotMass(Kilograms.of(Drive.ROBOT_MASS_KG))
          .withCustomModuleTranslations(Drive.getModuleTranslations())
          .withGyro(COTS.ofPigeon2())
          .withSwerveModules(
              new SwerveModuleSimulationConfig[] {
                new SwerveModuleSimulationConfig(
                    DCMotor.getKrakenX60(1),
                    DCMotor.getFalcon500(1),
                    TunerConstants.FrontLeft.DriveMotorGearRatio,
                    TunerConstants.FrontLeft.SteerMotorGearRatio,
                    Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                    Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                    Meters.of(TunerConstants.FrontLeft.WheelRadius),
                    KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                    Drive.WHEEL_COF),
                new SwerveModuleSimulationConfig(
                    DCMotor.getKrakenX60(1),
                    DCMotor.getFalcon500(1),
                    TunerConstants.FrontRight.DriveMotorGearRatio,
                    TunerConstants.FrontRight.SteerMotorGearRatio,
                    Volts.of(TunerConstants.FrontRight.DriveFrictionVoltage),
                    Volts.of(TunerConstants.FrontRight.SteerFrictionVoltage),
                    Meters.of(TunerConstants.FrontRight.WheelRadius),
                    KilogramSquareMeters.of(TunerConstants.FrontRight.SteerInertia),
                    Drive.WHEEL_COF),
                new SwerveModuleSimulationConfig(
                    DCMotor.getKrakenX60(1),
                    DCMotor.getFalcon500(1),
                    TunerConstants.BackLeft.DriveMotorGearRatio,
                    TunerConstants.BackLeft.SteerMotorGearRatio,
                    Volts.of(TunerConstants.BackLeft.DriveFrictionVoltage),
                    Volts.of(TunerConstants.BackLeft.SteerFrictionVoltage),
                    Meters.of(TunerConstants.BackLeft.WheelRadius),
                    KilogramSquareMeters.of(TunerConstants.BackLeft.SteerInertia),
                    Drive.WHEEL_COF),
                new SwerveModuleSimulationConfig(
                    DCMotor.getKrakenX60(1),
                    DCMotor.getFalcon500(1),
                    TunerConstants.BackRight.DriveMotorGearRatio,
                    TunerConstants.BackRight.SteerMotorGearRatio,
                    Volts.of(TunerConstants.BackRight.DriveFrictionVoltage),
                    Volts.of(TunerConstants.BackRight.SteerFrictionVoltage),
                    Meters.of(TunerConstants.BackRight.WheelRadius),
                    KilogramSquareMeters.of(TunerConstants.BackRight.SteerInertia),
                    Drive.WHEEL_COF)
              });
}
