package frc.robot26.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot26.subsystems.feeder.FeederConstants.GEARING_COMPLIANT_WHEEL;
import static frc.robot26.subsystems.feeder.FeederConstants.GEARING_FEEDER;
import static frc.robot26.subsystems.feeder.FeederConstants.Real;
import static frc.robot26.subsystems.feeder.FeederConstants.SUPPLY_CURRENT_LIMIT;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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

public class FeederIOTalonFX implements FeederIO {
  private final TalonFX lead, follower, compliantWheel;
  private final StatusSignal<Angle> leadPosition, compliantWheelPosition;
  private final StatusSignal<AngularVelocity> leadVelocity, compliantWheelVelocity;
  private final StatusSignal<Voltage> leadVoltage, compliantWheelVoltage;
  private final StatusSignal<Current> leadCurrent, compliantWheelCurrent;
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  public FeederIOTalonFX() {
    lead = new TalonFX(Real.leadMotorID);
    follower = new TalonFX(Real.followerMotorID);
    compliantWheel = new TalonFX(Real.compliantWheelID);
    leadVelocity = lead.getVelocity();
    leadPosition = lead.getPosition();
    leadVoltage = lead.getMotorVoltage();
    leadCurrent = lead.getStatorCurrent();
    compliantWheelVelocity = compliantWheel.getVelocity();
    compliantWheelPosition = compliantWheel.getPosition();
    compliantWheelVoltage = compliantWheel.getMotorVoltage();
    compliantWheelCurrent = compliantWheel.getStatorCurrent();

    var leadConfig = new TalonFXConfiguration();
    var compliantWheelConfig = new TalonFXConfiguration();

    leadConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leadConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    leadConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT.in(Amps);
    leadConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leadConfig.Feedback.SensorToMechanismRatio = GEARING_FEEDER;
    leadConfig.Voltage.PeakForwardVoltage = 10;
    leadConfig.Voltage.PeakReverseVoltage = -10;
    leadConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    compliantWheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    compliantWheelConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    compliantWheelConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT.in(Amps);
    compliantWheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    compliantWheelConfig.Feedback.SensorToMechanismRatio = GEARING_COMPLIANT_WHEEL;
    compliantWheelConfig.Voltage.PeakForwardVoltage = 10;
    compliantWheelConfig.Voltage.PeakReverseVoltage = -10;
    compliantWheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply feeder velocity PID gains to the lead Talon (tunable via dashboard)
    Real.feederPIDs.applyToTalonFXConfig(lead, leadConfig);
    lead.getConfigurator().apply(leadConfig, 0.25);
    lead.setPosition(0);
    follower.setControl(new Follower(Real.leadMotorID, MotorAlignmentValue.Opposed));

    compliantWheel.getConfigurator().apply(compliantWheelConfig, 0.25);
    compliantWheel.setPosition(0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        leadCurrent,
        leadVoltage,
        leadPosition,
        leadVelocity,
        compliantWheelCurrent,
        compliantWheelVoltage,
        compliantWheelPosition,
        compliantWheelVelocity);
    ParentDevice.optimizeBusUtilizationForAll(lead, follower, compliantWheel);
  }

  @Override
  public void setFeederOpenLoop(Voltage output) {
    lead.setVoltage(output.in(Volts));
    compliantWheel.setVoltage(output.in(Volts));
  }

  @Override
  public void setFeederClosedLoop(AngularVelocity velocity) {
    lead.setControl(velocityVoltageRequest.withVelocity(velocity));
    compliantWheel.setControl(velocityVoltageRequest.withVelocity(velocity));
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    var feederConnectedStatus =
        BaseStatusSignal.refreshAll(
            leadCurrent,
            leadVoltage,
            leadPosition,
            leadVelocity,
            compliantWheelCurrent,
            compliantWheelVoltage,
            compliantWheelPosition,
            compliantWheelVelocity);

    inputs.feederConnected = feederConnectedStatus.isOK();
    inputs.feederVelocity = leadVelocity.getValue();
    inputs.feederCurrent = leadCurrent.getValue();
    inputs.feederAppliedVolts = leadVoltage.getValue();

    inputs.compliantWheelVelocity = compliantWheelVelocity.getValue();
    inputs.compliantWheelCurrent = compliantWheelCurrent.getValue();
    inputs.compliantWheelAppliedVolts = compliantWheelVoltage.getValue();
  }
}
