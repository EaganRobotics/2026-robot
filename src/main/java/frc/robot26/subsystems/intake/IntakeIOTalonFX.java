package frc.robot26.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot26.subsystems.intake.IntakeConstants.GEARING;
import static frc.robot26.subsystems.intake.IntakeConstants.Real;
import static frc.robot26.subsystems.intake.IntakeConstants.SUPPLY_CURRENT_LIMIT;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
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

public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX lead, follower;
  private final StatusSignal<Angle> leadPosition;
  private final StatusSignal<AngularVelocity> leadVelocity;
  private final StatusSignal<Voltage> leadVoltage;
  private final StatusSignal<Current> leadCurrent;

  public IntakeIOTalonFX() {
    // TODO: ADD THIRD MOTOR

    lead = new TalonFX(Real.leadMotorID);
    follower = new TalonFX(Real.followerMotorID);

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

    lead.getConfigurator().apply(leadConfig, 0.25);
    lead.setPosition(0);
    follower.setControl(new Follower(Real.leadMotorID, MotorAlignmentValue.Opposed));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, leadCurrent, leadVoltage, leadPosition, leadVelocity);
    ParentDevice.optimizeBusUtilizationForAll(lead, follower);
  }

  @Override
  public void setOpenLoop(Voltage output) {
    lead.setVoltage(output.in(Volts));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var connectedStatus =
        BaseStatusSignal.refreshAll(leadCurrent, leadVoltage, leadPosition, leadVelocity);

    inputs.intakeConnected = connectedStatus.isOK();
    inputs.intakeVelocity = leadVelocity.getValue();
    inputs.intakeCurrent = leadCurrent.getValue();
    inputs.intakeAppliedVolts = leadVoltage.getValue();
  }
}
