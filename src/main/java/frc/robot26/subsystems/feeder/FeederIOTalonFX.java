package frc.robot26.subsystems.feeder;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot26.subsystems.feeder.FeederConstants.GEARING;
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
  private final StatusSignal<Angle> leadPosition;
  private final StatusSignal<AngularVelocity> leadVelocity;
  private final StatusSignal<Voltage> leadVoltage;
  private final StatusSignal<Current> leadCurrent;
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  public FeederIOTalonFX() {
    lead = new TalonFX(Real.leadMotorID);
    follower = new TalonFX(Real.followerMotorID);
    compliantWheel = new TalonFX(Real.compliantWheelID);
    leadVelocity = lead.getVelocity();
    leadPosition = lead.getPosition();
    leadVoltage = lead.getMotorVoltage();
    leadCurrent = lead.getStatorCurrent();

    var leadConfig = new TalonFXConfiguration();

    leadConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leadConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    leadConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT.in(Amps);
    leadConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leadConfig.Feedback.SensorToMechanismRatio = GEARING;
    leadConfig.Voltage.PeakForwardVoltage = 10;
    leadConfig.Voltage.PeakReverseVoltage = -10;
    leadConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    var compliantWheelConfig = new TalonFXConfiguration();

    compliantWheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    compliantWheelConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    compliantWheelConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT.in(Amps);
    compliantWheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    compliantWheelConfig.Feedback.SensorToMechanismRatio = GEARING;
    compliantWheelConfig.Voltage.PeakForwardVoltage = 10;
    compliantWheelConfig.Voltage.PeakReverseVoltage = -10;
    compliantWheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply feeder velocity PID gains to the lead Talon (tunable via dashboard)
    Real.feederPIDs.applyToTalonFXConfig(lead, leadConfig);
    lead.getConfigurator().apply(leadConfig, 0.25);
    lead.setPosition(0);
    follower.setControl(new Follower(Real.leadMotorID, MotorAlignmentValue.Opposed));
    // compliantWheel.setControl(new Follower(Real.leadMotorID, MotorAlignmentValue.Opposed));
    // Apply feeder velocity PID gains to the lead Talon (tunable via dashboard)
    Real.feederPIDs.applyToTalonFXConfig(compliantWheel, compliantWheelConfig);
    compliantWheel.getConfigurator().apply(compliantWheelConfig, 0.25);
    compliantWheel.setPosition(0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, leadCurrent, leadVoltage, leadPosition, leadVelocity);
    ParentDevice.optimizeBusUtilizationForAll(lead, follower, compliantWheel);
  }

  @Override
  public void setFeederOpenLoop(Voltage output) {
    lead.setVoltage(output.in(Volts));
  }

  @Override
  public void setFeederClosedLoop(AngularVelocity velocity) {
    lead.setControl(velocityVoltageRequest.withVelocity(velocity));
    compliantWheel.setControl(velocityVoltageRequest.withVelocity(velocity.times(.1)));
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    var feederConnectedStatus =
        BaseStatusSignal.refreshAll(leadCurrent, leadVoltage, leadPosition, leadVelocity);

    inputs.feederConnected = feederConnectedStatus.isOK();
    inputs.feederVelocity = leadVelocity.getValue();
    inputs.feederCurrent = leadCurrent.getValue();
    inputs.feederAppliedVolts = leadVoltage.getValue();
  }
}
