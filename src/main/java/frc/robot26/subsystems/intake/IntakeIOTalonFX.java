package frc.robot26.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot26.subsystems.intake.IntakeConstants.GEARING_DEPLOY;
import static frc.robot26.subsystems.intake.IntakeConstants.GEARING_INTAKE;
import static frc.robot26.subsystems.intake.IntakeConstants.Real;
import static frc.robot26.subsystems.intake.IntakeConstants.SUPPLY_CURRENT_LIMIT;
import static frc.robot26.subsystems.intake.IntakeConstants.deployRotationLimit;
import static frc.robot26.subsystems.intake.IntakeConstants.retractRotationLimit;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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
import frc.lib.devices.DigitalInputWrapper;
import frc.robot26.subsystems.intake.Intake.DeployState;

public class IntakeIOTalonFX implements IntakeIO {
  private final DigitalInputWrapper limitSwitch =
      new DigitalInputWrapper(3, "limitSwitch", true); // TODO: adjust
  private final TalonFX lead, follower, deploy;
  private final MotionMagicVoltage deployPositionRequest = new MotionMagicVoltage(0);
  private final StatusSignal<Angle> leadPosition, deployPosition;
  private final StatusSignal<AngularVelocity> leadVelocity, deployVelocity;
  private final StatusSignal<Voltage> leadVoltage, deployVoltage;
  private final StatusSignal<Current> leadCurrent, deployCurrent;

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

    leadConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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
    deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    Real.deployPIDs.applyToTalonFXConfig(deploy, deployConfig);

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

  @Override
  public void setDeployPosition(DeployState state) {
    switch (state) {
      case EXTENDED:
        deploy.setControl(deployPositionRequest.withPosition(deployRotationLimit.in(Rotations)));
        break;
      case RETRACTED:
        deploy.setControl(deployPositionRequest.withPosition(retractRotationLimit.in(Rotations)));
        break;
    }
  }
}
