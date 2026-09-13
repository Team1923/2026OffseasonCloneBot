package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.RollersConstants;

/** Rollers IO backed by a TalonFX motor controller. */
public class RollersIOTalonFX implements RollersIO {
  protected final TalonFX floorMotor = new TalonFX(RollersConstants.MOTOR_ID);

  private final StatusSignal<AngularVelocity> velocity = floorMotor.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = floorMotor.getMotorVoltage();
  private final StatusSignal<Current> supplyCurrent = floorMotor.getSupplyCurrent();
  private final StatusSignal<Current> statorCurrent = floorMotor.getStatorCurrent();
  private final StatusSignal<Temperature> temperature = floorMotor.getDeviceTemp();

  private final Debouncer connectedDebounce = new Debouncer(0.5);

  public RollersIOTalonFX() {
    floorMotor.getConfigurator().apply(RollersConstants.MOTOR_CONFIG);
  }

  @Override
  public void updateInputs(RollersIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            velocity, appliedVolts, supplyCurrent, statorCurrent, temperature);

    inputs.floorConnected = connectedDebounce.calculate(status.isOK());
    inputs.floorVelocityRpm = velocity.getValueAsDouble() * 60.0;
    inputs.floorAppliedVolts = appliedVolts.getValueAsDouble();
    inputs.floorSupplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.floorStatorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.floorTemperatureCelsius = temperature.getValueAsDouble();
  }

  @Override
  public void setFloorControl(ControlRequest request) {
    floorMotor.setControl(request);
  }
}
