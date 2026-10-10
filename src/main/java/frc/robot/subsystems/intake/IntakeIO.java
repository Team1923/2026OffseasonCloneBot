package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.IntakeConstants;
import frc.robot.lib.phoenix.PhoenixUtil;
import frc.robot.lib.sim.TalonFXMotorSim;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Hardware layer for the intake pivot and rollers.
 *
 * <ul>
 *   <li>{@link Real} talks to the TalonFXs on the robot.
 *   <li>{@link Sim} runs the same TalonFX code against simulated physics.
 *   <li>{@code new IntakeIO() {}} does nothing and is used for log replay.
 * </ul>
 */
public interface IntakeIO {
  /** Every sensor value the intake reads. Add new fields to {@code toLog} and {@code fromLog}. */
  class IntakeIOInputs implements LoggableInputs {
    public boolean armConnected = false;
    public double armPositionDegrees = 0.0;
    public double armVelocityDegreesPerSecond = 0.0;
    public double armAppliedVolts = 0.0;
    public double armSupplyCurrentAmps = 0.0;
    public double armStatorCurrentAmps = 0.0;
    public double armTemperatureCelsius = 0.0;

    public boolean rollerConnected = false;
    public double rollerVelocityRpm = 0.0;
    public double rollerAppliedVolts = 0.0;
    public double rollerSupplyCurrentAmps = 0.0;
    public double rollerStatorCurrentAmps = 0.0;
    public double rollerTemperatureCelsius = 0.0;

    @Override
    public void toLog(LogTable table) {
      table.put("ArmConnected", armConnected);
      table.put("ArmPositionDegrees", armPositionDegrees);
      table.put("ArmVelocityDegreesPerSecond", armVelocityDegreesPerSecond);
      table.put("ArmAppliedVolts", armAppliedVolts);
      table.put("ArmSupplyCurrentAmps", armSupplyCurrentAmps);
      table.put("ArmStatorCurrentAmps", armStatorCurrentAmps);
      table.put("ArmTemperatureCelsius", armTemperatureCelsius);

      table.put("RollerConnected", rollerConnected);
      table.put("RollerVelocityRpm", rollerVelocityRpm);
      table.put("RollerAppliedVolts", rollerAppliedVolts);
      table.put("RollerSupplyCurrentAmps", rollerSupplyCurrentAmps);
      table.put("RollerStatorCurrentAmps", rollerStatorCurrentAmps);
      table.put("RollerTemperatureCelsius", rollerTemperatureCelsius);
    }

    @Override
    public void fromLog(LogTable table) {
      armConnected = table.get("ArmConnected", armConnected);
      armPositionDegrees = table.get("ArmPositionDegrees", armPositionDegrees);
      armVelocityDegreesPerSecond =
          table.get("ArmVelocityDegreesPerSecond", armVelocityDegreesPerSecond);
      armAppliedVolts = table.get("ArmAppliedVolts", armAppliedVolts);
      armSupplyCurrentAmps = table.get("ArmSupplyCurrentAmps", armSupplyCurrentAmps);
      armStatorCurrentAmps = table.get("ArmStatorCurrentAmps", armStatorCurrentAmps);
      armTemperatureCelsius = table.get("ArmTemperatureCelsius", armTemperatureCelsius);

      rollerConnected = table.get("RollerConnected", rollerConnected);
      rollerVelocityRpm = table.get("RollerVelocityRpm", rollerVelocityRpm);
      rollerAppliedVolts = table.get("RollerAppliedVolts", rollerAppliedVolts);
      rollerSupplyCurrentAmps = table.get("RollerSupplyCurrentAmps", rollerSupplyCurrentAmps);
      rollerStatorCurrentAmps = table.get("RollerStatorCurrentAmps", rollerStatorCurrentAmps);
      rollerTemperatureCelsius = table.get("RollerTemperatureCelsius", rollerTemperatureCelsius);
    }
  }

  /** Reads the latest sensor values into {@code inputs}. */
  default void updateInputs(IntakeIOInputs inputs) {}

  /** Sends a Phoenix control request to the pivot motor. */
  default void setArmControl(ControlRequest request) {}

  /** Sends a Phoenix control request to the roller motor. */
  default void setRollerControl(ControlRequest request) {}

  /** Seeds the pivot encoder with a known mechanism angle. */
  default void setArmPosition(double degrees) {}

  /** Real intake hardware: two TalonFX motor controllers. */
  class Real implements IntakeIO {
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

    public Real() {
      PhoenixUtil.tryUntilOk(
          "Intake arm config", () -> arm.getConfigurator().apply(IntakeConstants.ARM_CONFIG));
      PhoenixUtil.tryUntilOk(
          "Intake roller config", () -> roller.getConfigurator().apply(IntakeConstants.ROLLER_CONFIG));
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
      inputs.armVelocityDegreesPerSecond =
          Units.rotationsToDegrees(armVelocity.getValueAsDouble());
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

  /** Simulated intake: the TalonFX control loops run in Phoenix sim against DC motor physics. */
  class Sim extends Real {
    private final TalonFXMotorSim armSim =
        new TalonFXMotorSim(
            arm,
            DCMotor.getKrakenX60(1),
            IntakeConstants.ARM_SIM_MOI_KG_METERS_SQUARED,
            IntakeConstants.ARM_GEAR_RATIO);
    private final TalonFXMotorSim rollerSim =
        new TalonFXMotorSim(
            roller,
            DCMotor.getKrakenX60(1),
            IntakeConstants.ROLLER_SIM_MOI_KG_METERS_SQUARED,
            IntakeConstants.ROLLER_GEAR_RATIO);

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
      armSim.update();
      rollerSim.update();
      super.updateInputs(inputs);
    }
  }
}
