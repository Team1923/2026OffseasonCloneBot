package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IntakeConstants;

/** Intake IO backed by two TalonFX motor controllers. */
public class IntakeIOTalonFX implements IntakeIO {
  protected final TalonFX arm = new TalonFX(IntakeConstants.ARM_MOTOR_ID);
  protected final TalonFX roller = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID);

  private final StatusSignal<Angle> armPosition = arm.getPosition();
  private final StatusSignal<AngularVelocity> armVelocity = arm.getVelocity();
  private final StatusSignal<Voltage> armAppliedVolts = arm.getMotorVoltage();
  private final StatusSignal<Current> armSupplyCurrent = arm.getSupplyCurrent();
  private final StatusSignal<Current> armStatorCurrent = arm.getStatorCurrent();
  private final StatusSignal<Temperature> armTemperature = arm.getDeviceTemp();

  private final StatusSignal<AngularVelocity> rollerVelocity = roller.getVelocity();
  private final StatusSignal<Voltage> rollerAppliedVolts = roller.getMotorVoltage();
  private final StatusSignal<Current> rollerSupplyCurrent = roller.getSupplyCurrent();
  private final StatusSignal<Current> rollerStatorCurrent = roller.getStatorCurrent();
  private final StatusSignal<Temperature> rollerTemperature = roller.getDeviceTemp();

  private final Debouncer armConnectedDebounce = new Debouncer(0.5);
  private final Debouncer rollerConnectedDebounce = new Debouncer(0.5);

  public IntakeIOTalonFX() {
    arm.getConfigurator().apply(IntakeConstants.ARM_CONFIG);
    roller.getConfigurator().apply(IntakeConstants.ROLLER_CONFIG);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var armStatus =
        BaseStatusSignal.refreshAll(
            armPosition,
            armVelocity,
            armAppliedVolts,
            armSupplyCurrent,
            armStatorCurrent,
            armTemperature);
    var rollerStatus =
        BaseStatusSignal.refreshAll(
            rollerVelocity,
            rollerAppliedVolts,
            rollerSupplyCurrent,
            rollerStatorCurrent,
            rollerTemperature);

    inputs.armConnected = armConnectedDebounce.calculate(armStatus.isOK());
    inputs.armPositionDegrees = Units.rotationsToDegrees(armPosition.getValueAsDouble());
    inputs.armVelocityDegreesPerSecond = Units.rotationsToDegrees(armVelocity.getValueAsDouble());
    inputs.armAppliedVolts = armAppliedVolts.getValueAsDouble();
    inputs.armSupplyCurrentAmps = armSupplyCurrent.getValueAsDouble();
    inputs.armStatorCurrentAmps = armStatorCurrent.getValueAsDouble();
    inputs.armTemperatureCelsius = armTemperature.getValueAsDouble();

    inputs.rollerConnected = rollerConnectedDebounce.calculate(rollerStatus.isOK());
    inputs.rollerVelocityRpm = rollerVelocity.getValueAsDouble() * 60.0;
    inputs.rollerAppliedVolts = rollerAppliedVolts.getValueAsDouble();
    inputs.rollerSupplyCurrentAmps = rollerSupplyCurrent.getValueAsDouble();
    inputs.rollerStatorCurrentAmps = rollerStatorCurrent.getValueAsDouble();
    inputs.rollerTemperatureCelsius = rollerTemperature.getValueAsDouble();
  }

  @Override
  public void setArmControl(ControlRequest request) {
    arm.setControl(request);
  }

  @Override
  public void setRollerControl(ControlRequest request) {
    roller.setControl(request);
  }

  @Override
  public void setArmPosition(double degrees) {
    arm.setPosition(Units.degreesToRotations(degrees));
  }
}
