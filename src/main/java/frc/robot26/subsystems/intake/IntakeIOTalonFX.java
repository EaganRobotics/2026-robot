package frc.robot26.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot26.subsystems.intake.IntakeConstants.GEARING_DEPLOY;
import static frc.robot26.subsystems.intake.IntakeConstants.GEARING_INTAKE;
import static frc.robot26.subsystems.intake.IntakeConstants.SUPPLY_CURRENT_LIMIT;
import static frc.robot26.subsystems.intake.IntakeConstants.deployRotationsFrom;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
import frc.lib.devices.DigitalInputWrapper;
import frc.robot26.subsystems.intake.IntakeConstants.Real;

public class IntakeIOTalonFX implements IntakeIO {
  private final DigitalInputWrapper limitSwitch =
      new DigitalInputWrapper(Real.limitSwitchChannel, "limitSwitch", true);
  private final TalonFX lead, follower, deploy;
  private final StatusSignal<Angle> leadPosition, deployPosition;
  private final StatusSignal<AngularVelocity> leadVelocity, deployVelocity;
  private final StatusSignal<Voltage> leadVoltage, deployVoltage;
  private final StatusSignal<Current> leadCurrent, deployCurrent;
  private final VelocityVoltage intakeVelocityVoltageRequest = new VelocityVoltage(0.0);
  private final PositionVoltage deployPositionVoltageRequest = new PositionVoltage(0.0);

  public IntakeIOTalonFX() {
    lead = new TalonFX(Real.leadMotorID);
    follower = new TalonFX(Real.followerMotorID);
    deploy = new TalonFX(Real.deployMotorID);

    leadVelocity = lead.getVelocity();
    leadPosition = lead.getPosition();
    leadVoltage = lead.getMotorVoltage();
    leadCurrent = lead.getStatorCurrent();

    deployVelocity = deploy.getVelocity();
    deployPosition = deploy.getPosition();
    deployVoltage = deploy.getMotorVoltage();
    deployCurrent = deploy.getStatorCurrent();

    var leadConfig = new TalonFXConfiguration();
    var deployConfig = new TalonFXConfiguration();

    leadConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    leadConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    leadConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT.in(Amps);
    leadConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leadConfig.Feedback.SensorToMechanismRatio = GEARING_INTAKE;
    leadConfig.Voltage.PeakForwardVoltage = 10;
    leadConfig.Voltage.PeakReverseVoltage = -10;
    leadConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    deployConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    deployConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    deployConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT.in(Amps);
    deployConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    deployConfig.Feedback.SensorToMechanismRatio = GEARING_DEPLOY;
    deployConfig.Voltage.PeakForwardVoltage = 10;
    deployConfig.Voltage.PeakReverseVoltage = -10;
    deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    deployConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    deployConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        deployRotationsFrom(IntakeConstants.deployLimit).in(Rotations);
    deployConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    deployConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        deployRotationsFrom(IntakeConstants.retractLimit).in(Rotations);

    Real.deployAcceleration.addListener(
        (acceleration) -> {
          deployConfig.MotionMagic.MotionMagicAcceleration =
              DegreesPerSecondPerSecond.of(acceleration).in(RotationsPerSecondPerSecond);
          deploy.getConfigurator().apply(deployConfig);
        });
    Real.deployCruiseVelocity.addListener(
        (velocity) -> {
          deployConfig.MotionMagic.MotionMagicCruiseVelocity =
              DegreesPerSecond.of(velocity).in(RotationsPerSecond);
          deploy.getConfigurator().apply(deployConfig);
        });

    Real.deployPIDs.applyToTalonFXConfig(deploy, deployConfig);
    Real.intakePIDs.applyToTalonFXConfig(lead, leadConfig);

    lead.getConfigurator().apply(leadConfig, 0.25);
    lead.setPosition(0);
    follower.setControl(new Follower(Real.leadMotorID, MotorAlignmentValue.Opposed));

    deploy.getConfigurator().apply(deployConfig, 0.25);
    deploy.setPosition(0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        leadCurrent,
        leadVoltage,
        leadPosition,
        leadVelocity,
        deployCurrent,
        deployVoltage,
        deployPosition,
        deployVelocity);
    ParentDevice.optimizeBusUtilizationForAll(lead, follower, deploy);
  }

  @Override
  public void setIntakeOpenLoop(Voltage output) {
    lead.setVoltage(output.in(Volts));
  }

  @Override
  public void setDeployOpenLoop(Voltage output) {
    deploy.setVoltage(output.in(Volts));
  }

  @Override
  public void setIntakeClosedLoop(AngularVelocity velocity) {
    lead.setControl(intakeVelocityVoltageRequest.withVelocity(velocity));
  }

  @Override
  public void setDeployClosedLoop(Angle angle) {
    deploy.setControl(deployPositionVoltageRequest.withPosition(angle));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var intakeConnectedStatus =
        BaseStatusSignal.refreshAll(leadCurrent, leadVoltage, leadPosition, leadVelocity);
    var deployConnectedStatus =
        BaseStatusSignal.refreshAll(deployCurrent, deployVoltage, deployPosition, deployVelocity);

    inputs.intakeConnected = intakeConnectedStatus.isOK();
    inputs.intakeVelocity = leadVelocity.getValue();
    inputs.intakeCurrent = leadCurrent.getValue();
    inputs.intakeAppliedVolts = leadVoltage.getValue();

    inputs.deployConnected = deployConnectedStatus.isOK();
    inputs.deployVelocity = deployVelocity.getValue();
    inputs.deployCurrent = deployCurrent.getValue();
    inputs.deployAppliedVolts = deployVoltage.getValue();
    inputs.deployPosition = deployPosition.getValue();

    inputs.limit = limitSwitch.get();
  }
}
