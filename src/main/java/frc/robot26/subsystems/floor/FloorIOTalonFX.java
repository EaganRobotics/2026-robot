package frc.robot26.subsystems.floor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot26.subsystems.floor.FloorConstants.GEARING;
import static frc.robot26.subsystems.floor.FloorConstants.SUPPLY_CURRENT_LIMIT;

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
import frc.robot26.subsystems.floor.FloorConstants.Real;

public class FloorIOTalonFX implements FloorIO {
  private final TalonFX lead;
  private final StatusSignal<Angle> leadPosition;
  private final StatusSignal<AngularVelocity> leadVelocity;
  private final StatusSignal<Voltage> leadVoltage;
  private final StatusSignal<Current> leadCurrent;

  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  public FloorIOTalonFX() {
    lead = new TalonFX(Real.leadMotorID);
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

    FloorConstants.floorPIDs.applyToTalonFXConfig(lead, leadConfig);

    lead.getConfigurator().apply(leadConfig, 0.25);
    lead.setPosition(0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, leadCurrent, leadVoltage, leadPosition, leadVelocity);
    ParentDevice.optimizeBusUtilizationForAll(lead);
  }

  @Override
  public void setFloorOpenLoop(Voltage output) {
    lead.setVoltage(output.in(Volts));
  }

  @Override
  public void setFloorClosedLoop(AngularVelocity velocity) {
    lead.setControl(velocityVoltageRequest.withVelocity(velocity));
  }

  @Override
  public void updateInputs(FloorIOInputs inputs) {
    var floorConnectedStatus =
        BaseStatusSignal.refreshAll(leadCurrent, leadVoltage, leadPosition, leadVelocity);

    inputs.floorConnected = floorConnectedStatus.isOK();
    inputs.floorVelocity = leadVelocity.getValue();
    inputs.floorCurrent = leadCurrent.getValue();
    inputs.floorAppliedVolts = leadVoltage.getValue();
  }
}
