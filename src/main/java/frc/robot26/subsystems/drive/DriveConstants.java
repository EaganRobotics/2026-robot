package frc.robot26.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.units.measure.*;

public class DriveConstants {

  // Stator current limits
  public static final Current DRIVE_CURRENT_LIMIT = Amps.of(40); // TODO: adjust
  public static final Current TURN_CURRENT_LIMIT = Amps.of(40); // TODO: adjust

  // The stator current at which the wheels start to slip;
  private static final Current kSlipCurrent = Amps.of(120.0); // TODO measure this

  // PathPlanner and Maple Sim config constants
  public static final Distance kWheelRadius = Inches.of(1.836); // last measured 2/16
  public static final double ROBOT_MASS_KG = 68.0389; // TODO: change, this is max weight rn
  public static final double ROBOT_MOI = 4.7523; // Lxx = 4.49, Lyy = 4.962, Lzz = 4.805; used avg
  // of the 3
  public static final double WHEEL_COF =
      2.255; // https://www.chiefdelphi.com/t/what-is-the-coefficient-of-friction-of-the-new-swerve-drive-specialties-molded-spike-grip-wheels/512813
  public static final double kDriveRatioR1 = 7.03;
  public static final double kDriveRatioR2 = 6.03;
  public static final double kDriveRatioR3 = 5.27;

  public static final double kDriveGearRatio = kDriveRatioR2; // Source: MK5i swerve module page
  // gearing
  private static final double kSteerGearRatio = 26.0; // Source: MK5i swerve module page
  public static final double kMaxDriveMotorRPM = 6000.0;
  public static final LinearVelocity kSpeedR1 = FeetPerSecond.of(14.9);
  public static final LinearVelocity kSpeedR2 = FeetPerSecond.of(17.4);
  public static final LinearVelocity kSpeedR3 = FeetPerSecond.of(19.9);
  public static final LinearVelocity kSpeedAt12Volts = kSpeedR2;

  // PID Gains for PathPlanner
  public static final PIDConstants PP_TRANSLATION_GAINS = new PIDConstants(1.3, 0.0, 0.01);
  public static final PIDConstants PP_ROTATION_GAINS = new PIDConstants(2.6, 0.0, 0.1);

  /* These Gains constants only affect real hardware */
  public static class Real {

    // The steer motor uses any SwerveModule.SteerRequestType control request with
    // the output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    public static final Slot0Configs STEER_GAINS =
        new Slot0Configs()
            .withKP(80)
            .withKI(10)
            .withKD(0.5)
            .withKS(0.1)
            .withKV(1.91)
            .withKA(0)
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    private static final double METERS_TO_ROTATIONS = 0.3912;
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    public static final Slot0Configs DRIVE_GAINS =
        new Slot0Configs()
            .withKP(2.8444 * METERS_TO_ROTATIONS)
            .withKI(0 * METERS_TO_ROTATIONS)
            .withKD(0 * METERS_TO_ROTATIONS)
            .withKS(0.19598)
            .withKV(0.77537)
            .withKA(0.058183 * METERS_TO_ROTATIONS);
  }

  /* These Gains constants only affect simulation */
  public static class Sim {

    /* Steer gains */
    public static final double STEER_KS = 0.1;
    public static final double STEER_KV = 1.91;
    public static final double STEER_KA = 0.0;
    public static final double STEER_KP = 10.0;
    public static final double STEER_KI = 0.0;
    public static final double STEER_KD = 0.5;

    /* Drive gains */
    public static final double DRIVE_KS = 0.0;
    public static final double DRIVE_KV = 0.124;
    public static final double DRIVE_KA = 0.0;
    public static final double DRIVE_KP = 0.1;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;

    /* These are both calculated off of MK4i Swerve Module CAD */
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.04);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);

    private static final Voltage kSteerFrictionVoltage = Volts.of(0.4); // suggested range 0.3 -
    // 0.5
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.4); // suggested range 0.6 -
    // 0.8
  }

  // The closed-loop output type to use for the steer motors;
  // This affects the PID/FF gains for the steer motors
  private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
  // The closed-loop output type to use for the drive motors;
  // This affects the PID/FF gains for the drive motors
  private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;

  // The type of motor used for the drive motor
  private static final DriveMotorArrangement kDriveMotorType =
      DriveMotorArrangement.TalonFX_Integrated;
  // The type of motor used for the drive motor
  private static final SteerMotorArrangement kSteerMotorType =
      SteerMotorArrangement.TalonFX_Integrated;

  // The remote sensor feedback type to use for the steer motors; we don't use
  // CANcoder so this is
  // irrelevant
  private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.RemoteCANcoder;

  // Initial configs for the drive and steer motors and the azimuth encoder; these
  // cannot be null.
  // Some configs will be overwritten; check the `with*InitialConfigs()` API
  // documentation.
  private static final TalonFXConfiguration driveInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(DRIVE_CURRENT_LIMIT)
                  .withStatorCurrentLimitEnable(true));
  // Swerve azimuth does not require much torque output, so we can set a
  // relatively low stator current limit to help avoid brownouts without impacting
  // performance.
  private static final TalonFXConfiguration steerInitialConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimit(TURN_CURRENT_LIMIT)
                  .withStatorCurrentLimitEnable(true));

  // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
  private static final Pigeon2Configuration pigeonConfigs =
      new Pigeon2Configuration()
          .withMountPose(new MountPoseConfigs().withMountPoseYaw(Degrees.of(180)));

  private static final int kPigeonId = 21;

  // CAN bus that the devices are located on;
  // All swerve devices must share the same CAN bus
  public static final CANBus kCANBus = new CANBus();

  // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
  // This may need to be tuned to your individual robot
  private static final double kCoupleRatio = 2;

  private static final boolean kInvertLeftSide = true;
  private static final boolean kInvertRightSide = true;

  // Aign with gears facing inside

  // Front Left - Encoder 1, Module 0
  private static final int kFrontLeftDriveMotorId = 8;
  private static final int kFrontLeftSteerMotorId = 4;
  private static final int kFrontLeftEncoderId = 1;
  private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.391 + 0.5);
  private static final boolean kFrontLeftSteerMotorInverted = true;
  private static final boolean kFrontLeftEncoderInverted = false;
  private static final Distance kFrontLeftXPos = Inches.of(-10.375);
  private static final Distance kFrontLeftYPos = Inches.of(10.375);

  // Front Right - Encoder 2, Module 1
  private static final int kFrontRightDriveMotorId = 5;
  private static final int kFrontRightSteerMotorId = 1;
  private static final int kFrontRightEncoderId = 2;
  private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.831);
  private static final boolean kFrontRightSteerMotorInverted = true;
  private static final boolean kFrontRightEncoderInverted = false;
  private static final Distance kFrontRightXPos = Inches.of(-10.375);
  private static final Distance kFrontRightYPos = Inches.of(-10.375);

  // Back Left - Encoder 0, Module 2
  private static final int kBackLeftDriveMotorId = 6;
  private static final int kBackLeftSteerMotorId = 2;
  private static final int kBackLeftEncoderId = 0;
  private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.198);
  private static final boolean kBackLeftSteerMotorInverted = true;
  private static final boolean kBackLeftEncoderInverted = false;
  private static final Distance kBackLeftXPos = Inches.of(10.375);
  private static final Distance kBackLeftYPos = Inches.of(10.375);

  // Back Right - Encoder 3, Module 3
  private static final int kBackRightDriveMotorId = 7;
  private static final int kBackRightSteerMotorId = 3;
  private static final int kBackRightEncoderId = 3;
  private static final Angle kBackRightEncoderOffset = Rotations.of(-0.173);
  private static final boolean kBackRightSteerMotorInverted = true;
  private static final boolean kBackRightEncoderInverted = false;
  private static final Distance kBackRightXPos = Inches.of(10.375);
  private static final Distance kBackRightYPos = Inches.of(-10.375);

  public static final SwerveDrivetrainConstants DrivetrainConstants =
      new SwerveDrivetrainConstants()
          .withCANBusName(kCANBus.getName())
          .withPigeon2Id(kPigeonId)
          .withPigeon2Configs(pigeonConfigs);

  private static final SwerveModuleConstantsFactory<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      ConstantCreator =
          new SwerveModuleConstantsFactory<
                  TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
              .withDriveMotorGearRatio(kDriveGearRatio)
              .withSteerMotorGearRatio(kSteerGearRatio)
              .withCouplingGearRatio(kCoupleRatio)
              .withWheelRadius(kWheelRadius)
              .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
              .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
              .withSlipCurrent(kSlipCurrent)
              .withSpeedAt12Volts(kSpeedAt12Volts)
              .withDriveMotorType(kDriveMotorType)
              .withSteerMotorType(kSteerMotorType)
              .withFeedbackSource(kSteerFeedbackType)
              .withDriveMotorInitialConfigs(driveInitialConfigs)
              .withSteerMotorInitialConfigs(steerInitialConfigs)
              .withSteerInertia(Sim.kSteerInertia)
              .withDriveInertia(Sim.kDriveInertia)
              .withSteerFrictionVoltage(Sim.kSteerFrictionVoltage)
              .withDriveFrictionVoltage(Sim.kDriveFrictionVoltage)
              .withDriveMotorGains(Real.DRIVE_GAINS)
              .withSteerMotorGains(Real.STEER_GAINS)
              .withEncoderInitialConfigs(new CANcoderConfiguration());

  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft =
          ConstantCreator.createModuleConstants(
              kFrontLeftSteerMotorId,
              kFrontLeftDriveMotorId,
              kFrontLeftEncoderId,
              kFrontLeftEncoderOffset,
              kFrontLeftXPos,
              kFrontLeftYPos,
              kInvertLeftSide,
              kFrontLeftSteerMotorInverted,
              kFrontLeftEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight =
          ConstantCreator.createModuleConstants(
              kFrontRightSteerMotorId,
              kFrontRightDriveMotorId,
              kFrontRightEncoderId,
              kFrontRightEncoderOffset,
              kFrontRightXPos,
              kFrontRightYPos,
              kInvertRightSide,
              kFrontRightSteerMotorInverted,
              kFrontRightEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft =
          ConstantCreator.createModuleConstants(
              kBackLeftSteerMotorId,
              kBackLeftDriveMotorId,
              kBackLeftEncoderId,
              kBackLeftEncoderOffset,
              kBackLeftXPos,
              kBackLeftYPos,
              kInvertLeftSide,
              kBackLeftSteerMotorInverted,
              kBackLeftEncoderInverted);
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight =
          ConstantCreator.createModuleConstants(
              kBackRightSteerMotorId,
              kBackRightDriveMotorId,
              kBackRightEncoderId,
              kBackRightEncoderOffset,
              kBackRightXPos,
              kBackRightYPos,
              kInvertRightSide,
              kBackRightSteerMotorInverted,
              kBackRightEncoderInverted);

  // For max angular velocity calculations
  public static final Distance DRIVE_BASE_RADIUS =
      Meters.of(
          Math.max(
              Math.max(
                  Math.hypot(
                      DriveConstants.FrontLeft.LocationX, DriveConstants.FrontRight.LocationY),
                  Math.hypot(
                      DriveConstants.FrontRight.LocationX, DriveConstants.FrontRight.LocationY)),
              Math.max(
                  Math.hypot(DriveConstants.BackLeft.LocationX, DriveConstants.BackLeft.LocationY),
                  Math.hypot(
                      DriveConstants.BackRight.LocationX, DriveConstants.BackRight.LocationY))));

  public static final AngularVelocity MAX_ANGULAR_VELOCITY =
      RadiansPerSecond.of(kSpeedAt12Volts.in(MetersPerSecond) / DRIVE_BASE_RADIUS.in(Meters));

  public static final double ODOMETRY_FREQUENCY =
      new CANBus(DriveConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
}
