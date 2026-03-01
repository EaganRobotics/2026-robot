package frc.robot26.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot26.subsystems.shooter.ShooterConstants.GEARING_HOOD;
import static frc.robot26.subsystems.shooter.ShooterConstants.GEARING_SHOOTER;
import static frc.robot26.subsystems.shooter.ShooterConstants.SUPPLY_CURRENT_LIMIT;
import static frc.robot26.subsystems.shooter.ShooterConstants.hoodRotationEndLimit;
import static frc.robot26.subsystems.shooter.ShooterConstants.hoodRotationStartLimit;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot26.subsystems.shooter.ShooterConstants.Real;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX leadLeft, followerLeft, leadRight, followerRight, hood;
  private final MotionMagicVoltage hoodPositionRequest = new MotionMagicVoltage(0);
  private final MotionMagicVelocityVoltage velocityVoltageRequest =
      new MotionMagicVelocityVoltage(0.0);
  private final StatusSignal<Angle> leadPositionLeft, leadPositionRight, hoodPosition;
  private final StatusSignal<AngularVelocity> leadVelocityLeft, leadVelocityRight, hoodVelocity;
  private final StatusSignal<Voltage> leadVoltageLeft, leadVoltageRight, hoodVoltage;
  private final StatusSignal<Current> leadCurrentLeft, leadCurrentRight, hoodCurrent;

  public ShooterIOTalonFX() {
    leadLeft = new TalonFX(Real.leadLeftMotorID);
    followerLeft = new TalonFX(Real.followerLeftMotorID);
    leadRight = new TalonFX(Real.leadRightMotorID);
    followerRight = new TalonFX(Real.followerRightMotorID);
    hood = new TalonFX(Real.hoodMotorID);

    leadVelocityLeft = leadLeft.getVelocity();
    leadPositionLeft = leadLeft.getPosition();
    leadVoltageLeft = leadLeft.getMotorVoltage();
    leadCurrentLeft = leadLeft.getStatorCurrent();

    leadVelocityRight = leadRight.getVelocity();
    leadPositionRight = leadRight.getPosition();
    leadVoltageRight = leadRight.getMotorVoltage();
    leadCurrentRight = leadRight.getStatorCurrent();

    hoodVelocity = hood.getVelocity();
    hoodPosition = hood.getPosition();
    hoodVoltage = hood.getMotorVoltage();
    hoodCurrent = hood.getStatorCurrent();

    var leadConfigLeft = new TalonFXConfiguration();
    var leadConfigRight = new TalonFXConfiguration();
    var hoodConfig = new TalonFXConfiguration();

    leadConfigLeft.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leadConfigLeft.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    leadConfigLeft.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT.in(Amps);
    leadConfigLeft.CurrentLimits.SupplyCurrentLimitEnable = true;
    leadConfigLeft.Feedback.SensorToMechanismRatio = GEARING_SHOOTER;
    leadConfigLeft.Voltage.PeakForwardVoltage = 10;
    leadConfigLeft.Voltage.PeakReverseVoltage = -10;
    leadConfigLeft.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    leadConfigRight.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leadConfigRight.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    leadConfigRight.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT.in(Amps);
    leadConfigRight.CurrentLimits.SupplyCurrentLimitEnable = true;
    leadConfigRight.Feedback.SensorToMechanismRatio = GEARING_SHOOTER;
    leadConfigRight.Voltage.PeakForwardVoltage = 10;
    leadConfigRight.Voltage.PeakReverseVoltage = -10;
    leadConfigRight.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    hoodConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    hoodConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT.in(Amps);
    hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    hoodConfig.Feedback.SensorToMechanismRatio = GEARING_HOOD;
    hoodConfig.Voltage.PeakForwardVoltage = 10;
    hoodConfig.Voltage.PeakReverseVoltage = -10;
    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    hoodConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = hoodRotationEndLimit.in(Rotations);
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    hoodConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = hoodRotationStartLimit.in(Rotations);

    // Apply velocity-specific PID gains to the shooter Talons (tunable via dashboard)
    Real.shooterPIDs.applyToTalonFXConfig(leadLeft, leadConfigLeft);
    Real.shooterPIDs.applyToTalonFXConfig(leadRight, leadConfigRight);
    Real.hoodPIDs.applyToTalonFXConfig(hood, hoodConfig);
    Real.shooterAcceleration.addListener(
        (acceleration) -> {
          leadConfigLeft.MotionMagic.MotionMagicAcceleration = acceleration / 60.0;
          leadConfigRight.MotionMagic.MotionMagicAcceleration = acceleration / 60.0;
          leadLeft.getConfigurator().apply(leadConfigLeft);
          leadRight.getConfigurator().apply(leadConfigRight);
        });
    leadLeft.getConfigurator().apply(leadConfigLeft, 0.25);
    leadLeft.setPosition(0);
    followerLeft.setControl(new Follower(Real.leadLeftMotorID, MotorAlignmentValue.Aligned));

    leadRight.getConfigurator().apply(leadConfigRight, 0.25);
    leadRight.setPosition(0);
    followerRight.setControl(new Follower(Real.leadRightMotorID, MotorAlignmentValue.Aligned));

    hood.getConfigurator().apply(hoodConfig, 0.25);
    hood.setPosition(0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        leadCurrentLeft,
        leadVoltageLeft,
        leadPositionLeft,
        leadVelocityLeft,
        leadCurrentRight,
        leadVoltageRight,
        leadPositionRight,
        leadVelocityRight,
        hoodCurrent,
        hoodVoltage,
        hoodPosition,
        hoodVelocity);
    ParentDevice.optimizeBusUtilizationForAll(
        leadLeft, followerLeft, leadRight, followerRight, hood);
  }

  @Override
  public void setShooterOpenLoop(Voltage output) {
    leadLeft.setVoltage(output.in(Volts));
    leadRight.setVoltage(output.in(Volts));
  }

  @Override
  public void setShooterClosedLoop(AngularVelocity velocity) {
    leadLeft.setControl(velocityVoltageRequest.withVelocity(velocity));
    leadRight.setControl(velocityVoltageRequest.withVelocity(velocity));
  }

  @Override
  public void setHoodOpenLoop(Voltage output) {
    hood.setVoltage(output.in(Volts));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var shooterConnectedStatus =
        BaseStatusSignal.refreshAll(
            leadCurrentLeft,
            leadVoltageLeft,
            leadPositionLeft,
            leadVelocityLeft,
            leadCurrentRight,
            leadVoltageRight,
            leadPositionRight,
            leadVelocityRight);
    var hoodConnectedStatus =
        BaseStatusSignal.refreshAll(hoodCurrent, hoodVoltage, hoodPosition, hoodVelocity);

    inputs.shooterConnected = shooterConnectedStatus.isOK();
    inputs.shooterVelocity = leadVelocityLeft.getValue();
    inputs.shooterCurrent = leadCurrentLeft.getValue();
    inputs.shooterAppliedVolts = leadVoltageLeft.getValue();

    inputs.hoodConnected = hoodConnectedStatus.isOK();
    inputs.hoodVelocity = hoodVelocity.getValue();
    inputs.hoodCurrent = hoodCurrent.getValue();
    inputs.hoodAppliedVolts = hoodVoltage.getValue();
    inputs.hoodPosition = hoodPosition.getValue();
  }

  @Override
  public void setHoodPosition(Angle angle) {
    hood.setControl(hoodPositionRequest.withPosition(angle));
  }
}
