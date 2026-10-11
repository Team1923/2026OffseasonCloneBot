package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.RollersConstants;
import frc.robot.lib.sim.TalonFXMotorSim;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Hardware layer for the hopper floor roller.
 *
 * <ul>
 *   <li>{@link Real} talks to the floor TalonFX and its two feeder followers on the robot.
 *   <li>{@link Sim} runs the same TalonFX code against simulated physics.
 *   <li>{@code new RollersIO() {}} does nothing and is used for log replay.
 * </ul>
 */
public interface RollersIO {
  /** Feeder motor order used by every feeder array input: feeder one, feeder two. */
  int FEEDER_MOTOR_COUNT = 2;

  /** Every sensor value the rollers read. Add new fields to {@code toLog} and {@code fromLog}. */
  class RollersIOInputs implements LoggableInputs {
    public boolean floorConnected = false;
    public double floorVelocityRpm = 0.0;
    public double floorAppliedVolts = 0.0;
    public double floorSupplyCurrentAmps = 0.0;
    public double floorStatorCurrentAmps = 0.0;
    public double floorTemperatureCelsius = 0.0;

    public boolean[] feederConnected = new boolean[FEEDER_MOTOR_COUNT];
    public double[] feederAppliedVolts = new double[FEEDER_MOTOR_COUNT];
    public double[] feederSupplyCurrentAmps = new double[FEEDER_MOTOR_COUNT];
    public double[] feederStatorCurrentAmps = new double[FEEDER_MOTOR_COUNT];
    public double[] feederTemperatureCelsius = new double[FEEDER_MOTOR_COUNT];

    @Override
    public void toLog(LogTable table) {
      table.put("FloorConnected", floorConnected);
      table.put("FloorVelocityRpm", floorVelocityRpm);
      table.put("FloorAppliedVolts", floorAppliedVolts);
      table.put("FloorSupplyCurrentAmps", floorSupplyCurrentAmps);
      table.put("FloorStatorCurrentAmps", floorStatorCurrentAmps);
      table.put("FloorTemperatureCelsius", floorTemperatureCelsius);

      table.put("FeederConnected", feederConnected);
      table.put("FeederAppliedVolts", feederAppliedVolts);
      table.put("FeederSupplyCurrentAmps", feederSupplyCurrentAmps);
      table.put("FeederStatorCurrentAmps", feederStatorCurrentAmps);
      table.put("FeederTemperatureCelsius", feederTemperatureCelsius);
    }

    @Override
    public void fromLog(LogTable table) {
      floorConnected = table.get("FloorConnected", floorConnected);
      floorVelocityRpm = table.get("FloorVelocityRpm", floorVelocityRpm);
      floorAppliedVolts = table.get("FloorAppliedVolts", floorAppliedVolts);
      floorSupplyCurrentAmps = table.get("FloorSupplyCurrentAmps", floorSupplyCurrentAmps);
      floorStatorCurrentAmps = table.get("FloorStatorCurrentAmps", floorStatorCurrentAmps);
      floorTemperatureCelsius = table.get("FloorTemperatureCelsius", floorTemperatureCelsius);

      feederConnected = table.get("FeederConnected", feederConnected);
      feederAppliedVolts = table.get("FeederAppliedVolts", feederAppliedVolts);
      feederSupplyCurrentAmps = table.get("FeederSupplyCurrentAmps", feederSupplyCurrentAmps);
      feederStatorCurrentAmps = table.get("FeederStatorCurrentAmps", feederStatorCurrentAmps);
      feederTemperatureCelsius = table.get("FeederTemperatureCelsius", feederTemperatureCelsius);
    }
  }

  /** Reads the latest sensor values into {@code inputs}. */
  default void updateInputs(RollersIOInputs inputs) {}

  /** Sends a Phoenix control request to the floor roller motor. */
  default void setFloorControl(ControlRequest request) {}

  /**
   * Enables or disables the feeder motors. When enabled they follow the floor roller; when
   * disabled they are held in neutral so pieces are not pushed into the shooter.
   */
  default void setFeedersEnabled(boolean enabled) {}

  /** Real rollers hardware: one floor-roller leader and two feeder followers. */
  class Real implements RollersIO {
    protected final TalonFX floorMotor = new TalonFX(RollersConstants.MOTOR_ID);
    protected final TalonFX feederMotorOne = new TalonFX(RollersConstants.FEEDER_MOTOR_ONE_ID);
    protected final TalonFX feederMotorTwo = new TalonFX(RollersConstants.FEEDER_MOTOR_TWO_ID);

    /** Same order as the feeder array inputs. */
    protected final TalonFX[] feederMotors = {feederMotorOne, feederMotorTwo};

    // TODO: Verify each feeder motor's physical direction before enabling the mechanism.
    private final Follower[] feederFollowRequests = {
      new Follower(floorMotor.getDeviceID(), MotorAlignmentValue.Aligned),
      new Follower(floorMotor.getDeviceID(), MotorAlignmentValue.Aligned)
    };
    private final NeutralOut feederNeutralRequest = new NeutralOut();

    private final StatusSignal<AngularVelocity> velocity = floorMotor.getVelocity();
    private final StatusSignal<Voltage> appliedVolts = floorMotor.getMotorVoltage();
    private final StatusSignal<Current> supplyCurrent = floorMotor.getSupplyCurrent();
    private final StatusSignal<Current> statorCurrent = floorMotor.getStatorCurrent();
    private final StatusSignal<Temperature> temperature = floorMotor.getDeviceTemp();

    private final List<StatusSignal<Voltage>> feederAppliedVolts =
        List.of(feederMotorOne.getMotorVoltage(), feederMotorTwo.getMotorVoltage());
    private final List<StatusSignal<Current>> feederSupplyCurrent =
        List.of(feederMotorOne.getSupplyCurrent(), feederMotorTwo.getSupplyCurrent());
    private final List<StatusSignal<Current>> feederStatorCurrent =
        List.of(feederMotorOne.getStatorCurrent(), feederMotorTwo.getStatorCurrent());
    private final List<StatusSignal<Temperature>> feederTemperature =
        List.of(feederMotorOne.getDeviceTemp(), feederMotorTwo.getDeviceTemp());

    private final Debouncer connectedDebounce = new Debouncer(0.5);
    private final Debouncer[] feederConnectedDebounce = {new Debouncer(0.5), new Debouncer(0.5)};

    public Real() {
      floorMotor.getConfigurator().apply(RollersConstants.MOTOR_CONFIG);
      feederMotorOne.getConfigurator().apply(RollersConstants.FEEDER_MOTOR_CONFIG);
      feederMotorTwo.getConfigurator().apply(RollersConstants.FEEDER_MOTOR_CONFIG);

      setFeedersEnabled(false);
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

      for (int i = 0; i < FEEDER_MOTOR_COUNT; i++) {
        var feederStatus =
            BaseStatusSignal.refreshAll(
                feederAppliedVolts.get(i),
                feederSupplyCurrent.get(i),
                feederStatorCurrent.get(i),
                feederTemperature.get(i));
        inputs.feederConnected[i] = feederConnectedDebounce[i].calculate(feederStatus.isOK());
        inputs.feederAppliedVolts[i] = feederAppliedVolts.get(i).getValueAsDouble();
        inputs.feederSupplyCurrentAmps[i] = feederSupplyCurrent.get(i).getValueAsDouble();
        inputs.feederStatorCurrentAmps[i] = feederStatorCurrent.get(i).getValueAsDouble();
        inputs.feederTemperatureCelsius[i] = feederTemperature.get(i).getValueAsDouble();
      }
    }

    @Override
    public void setFloorControl(ControlRequest request) {
      floorMotor.setControl(request);
    }

    @Override
    public void setFeedersEnabled(boolean enabled) {
      for (int i = 0; i < FEEDER_MOTOR_COUNT; i++) {
        feederMotors[i].setControl(enabled ? feederFollowRequests[i] : feederNeutralRequest);
      }
    }
  }

  /**
   * Simulated rollers: the floor TalonFX velocity loop runs against DC motor physics. The feeders
   * only get a supply voltage so their follower output is visible; they are not in the physics.
   */
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
      for (TalonFX feederMotor : feederMotors) {
        feederMotor.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
      }
      super.updateInputs(inputs);
    }
  }
}
