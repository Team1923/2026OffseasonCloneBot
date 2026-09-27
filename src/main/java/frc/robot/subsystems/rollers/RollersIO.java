package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.RollersConstants;
import frc.robot.lib.sim.TalonFXMotorSim;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Hardware layer for the hopper floor roller.
 *
 * <ul>
 *   <li>{@link Real} talks to the TalonFX on the robot.
 *   <li>{@link Sim} runs the same TalonFX code against simulated physics.
 *   <li>{@code new RollersIO() {}} does nothing and is used for log replay.
 * </ul>
 */
public interface RollersIO {
  /** Every sensor value the rollers read. Add new fields to {@code toLog} and {@code fromLog}. */
  class RollersIOInputs implements LoggableInputs {
    public boolean floorConnected = false;
    public boolean pieceDetected = false;
    public double floorVelocityRpm = 0.0;
    public double floorAppliedVolts = 0.0;
    public double floorSupplyCurrentAmps = 0.0;
    public double floorStatorCurrentAmps = 0.0;
    public double floorTemperatureCelsius = 0.0;

    @Override
    public void toLog(LogTable table) {
      table.put("FloorConnected", floorConnected);
      table.put("PieceDetected", pieceDetected);
      table.put("FloorVelocityRpm", floorVelocityRpm);
      table.put("FloorAppliedVolts", floorAppliedVolts);
      table.put("FloorSupplyCurrentAmps", floorSupplyCurrentAmps);
      table.put("FloorStatorCurrentAmps", floorStatorCurrentAmps);
      table.put("FloorTemperatureCelsius", floorTemperatureCelsius);
    }

    @Override
    public void fromLog(LogTable table) {
      floorConnected = table.get("FloorConnected", floorConnected);
      pieceDetected = table.get("PieceDetected", pieceDetected);
      floorVelocityRpm = table.get("FloorVelocityRpm", floorVelocityRpm);
      floorAppliedVolts = table.get("FloorAppliedVolts", floorAppliedVolts);
      floorSupplyCurrentAmps = table.get("FloorSupplyCurrentAmps", floorSupplyCurrentAmps);
      floorStatorCurrentAmps = table.get("FloorStatorCurrentAmps", floorStatorCurrentAmps);
      floorTemperatureCelsius = table.get("FloorTemperatureCelsius", floorTemperatureCelsius);
    }
  }

  /** Reads the latest sensor values into {@code inputs}. */
  default void updateInputs(RollersIOInputs inputs) {}

  /** Sends a Phoenix control request to the floor roller motor. */
  default void setFloorControl(ControlRequest request) {}

  /** Real rollers hardware: one TalonFX motor controller. */
  class Real implements RollersIO {
    protected final TalonFX floorMotor = new TalonFX(RollersConstants.MOTOR_ID);
    protected final DigitalInput beamBreak =
        new DigitalInput(RollersConstants.BEAM_BREAK_DIO_PORT);

    private final StatusSignal<AngularVelocity> velocity = floorMotor.getVelocity();
    private final StatusSignal<Voltage> appliedVolts = floorMotor.getMotorVoltage();
    private final StatusSignal<Current> supplyCurrent = floorMotor.getSupplyCurrent();
    private final StatusSignal<Current> statorCurrent = floorMotor.getStatorCurrent();
    private final StatusSignal<Temperature> temperature = floorMotor.getDeviceTemp();

    private final Debouncer connectedDebounce = new Debouncer(0.5);

    public Real() {
      floorMotor.getConfigurator().apply(RollersConstants.MOTOR_CONFIG);
    }

    @Override
    public void updateInputs(RollersIOInputs inputs) {
      var status =
          BaseStatusSignal.refreshAll(
              velocity, appliedVolts, supplyCurrent, statorCurrent, temperature);

      inputs.floorConnected = connectedDebounce.calculate(status.isOK());
      inputs.pieceDetected = !beamBreak.get(); // TODO: verify polarity on the real sensor
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

  /** Simulated rollers: the TalonFX velocity loop runs in Phoenix sim against DC motor physics. */
  class Sim extends Real {
    private final TalonFXMotorSim floorSim =
        new TalonFXMotorSim(
            floorMotor,
            DCMotor.getKrakenX60(1),
            RollersConstants.SIM_MOI_KG_METERS_SQUARED,
            RollersConstants.GEAR_RATIO);

    @Override
    public void updateInputs(RollersIOInputs inputs) {
      floorSim.update();
      super.updateInputs(inputs);
    }
  }
}
