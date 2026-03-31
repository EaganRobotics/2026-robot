package frc.robot26.subsystems.greenRollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot26.subsystems.greenRollers.GreenRollersConstants.GEARING;
import static frc.robot26.subsystems.greenRollers.GreenRollersConstants.SUPPLY_CURRENT_LIMIT;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot26.subsystems.greenRollers.GreenRollersConstants.Real;
import frc.robot26.subsystems.greenRollers.GreenRollersIO.GreenRollersIOInputs;

public class GreenRollersIOTalonFX implements GreenRollersIO {
  private final TalonFX lead;
  private final StatusSignal<Angle> leadPosition;
  private final StatusSignal<AngularVelocity> leadVelocity;
  private final StatusSignal<Voltage> leadVoltage;
  private final StatusSignal<Current> leadCurrent;

  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  public GreenRollersIOTalonFX() {
    lead = new TalonFX(Real.greenRollersMotorID);
    leadVelocity = lead.getVelocity();
    leadPosition = lead.getPosition();
    leadVoltage = lead.getMotorVoltage();
    leadCurrent = lead.getStatorCurrent();

    var leadConfig = new TalonFXConfiguration();

    leadConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leadConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    leadConfig.CurrentLimits.SupplyCurrentLimit = SUPPLY_CURRENT_LIMIT.in(Amps);
    leadConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
    leadConfig.Feedback.SensorToMechanismRatio = GEARING;
    leadConfig.Voltage.PeakForwardVoltage = 12;
    leadConfig.Voltage.PeakReverseVoltage = -12;
    leadConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    GreenRollersConstants.greenRollersPIDs.applyToTalonFXConfig(lead, leadConfig);

    lead.getConfigurator().apply(leadConfig, 0.25);
    lead.setPosition(0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, leadCurrent, leadVoltage, leadPosition, leadVelocity);
    ParentDevice.optimizeBusUtilizationForAll(lead);
  }

  @Override
  public void setGreenRollersOpenLoop(Voltage output) {
    lead.setVoltage(output.in(Volts));
  }

  @Override
  public void setGreenRollersClosedLoop(AngularVelocity velocity) {
    lead.setControl(velocityVoltageRequest.withVelocity(velocity));
  }

  @Override
  public void updateInputs(GreenRollersIOInputs inputs) {
    var greenRollersConnectedStatus =
        BaseStatusSignal.refreshAll(leadCurrent, leadVoltage, leadPosition, leadVelocity);

    inputs.greenRollersConnected = greenRollersConnectedStatus.isOK();
    inputs.greenRollersVelocity = leadVelocity.getValue();
    inputs.greenRollersCurrent = leadCurrent.getValue();
    inputs.greenRollersAppliedVolts = leadVoltage.getValue();
  }
}
