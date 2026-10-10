package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lib.phoenix.PhoenixUtil;
import frc.robot.lib.sim.TalonFXMotorSim;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Hardware layer for the shooter hood and four-motor drum.
 *
 * <ul>
 *   <li>{@link Real} talks to the TalonFXs on the robot.
 *   <li>{@link Sim} runs the same TalonFX code against simulated physics.
 *   <li>{@code new ShooterIO() {}} does nothing and is used for log replay.
 * </ul>
 */
public interface ShooterIO {
  /** Drum motor order used by every drum array input: leader, follower one, two, three. */
  int DRUM_MOTOR_COUNT = 4;

  /** Every sensor value the shooter reads. Add new fields to {@code toLog} and {@code fromLog}. */
  class ShooterIOInputs implements LoggableInputs {
    public boolean hoodConnected = false;
    public double hoodPositionDegrees = 0.0;
    public double hoodVelocityDegreesPerSecond = 0.0;
    public double hoodAppliedVolts = 0.0;
    public double hoodSupplyCurrentAmps = 0.0;
    public double hoodStatorCurrentAmps = 0.0;
    public double hoodTemperatureCelsius = 0.0;

    /** Leader drum velocity; the followers mirror the leader. */
    public double drumVelocityRpm = 0.0;

    public boolean[] drumConnected = new boolean[DRUM_MOTOR_COUNT];
    public double[] drumAppliedVolts = new double[DRUM_MOTOR_COUNT];
    public double[] drumSupplyCurrentAmps = new double[DRUM_MOTOR_COUNT];
    public double[] drumStatorCurrentAmps = new double[DRUM_MOTOR_COUNT];
    public double[] drumTemperatureCelsius = new double[DRUM_MOTOR_COUNT];

    @Override
    public void toLog(LogTable table) {
      table.put("HoodConnected", hoodConnected);
      table.put("HoodPositionDegrees", hoodPositionDegrees);
      table.put("HoodVelocityDegreesPerSecond", hoodVelocityDegreesPerSecond);
      table.put("HoodAppliedVolts", hoodAppliedVolts);
      table.put("HoodSupplyCurrentAmps", hoodSupplyCurrentAmps);
      table.put("HoodStatorCurrentAmps", hoodStatorCurrentAmps);
      table.put("HoodTemperatureCelsius", hoodTemperatureCelsius);

      table.put("DrumVelocityRpm", drumVelocityRpm);
      table.put("DrumConnected", drumConnected);
      table.put("DrumAppliedVolts", drumAppliedVolts);
      table.put("DrumSupplyCurrentAmps", drumSupplyCurrentAmps);
      table.put("DrumStatorCurrentAmps", drumStatorCurrentAmps);
      table.put("DrumTemperatureCelsius", drumTemperatureCelsius);
    }

    @Override
    public void fromLog(LogTable table) {
      hoodConnected = table.get("HoodConnected", hoodConnected);
      hoodPositionDegrees = table.get("HoodPositionDegrees", hoodPositionDegrees);
      hoodVelocityDegreesPerSecond =
          table.get("HoodVelocityDegreesPerSecond", hoodVelocityDegreesPerSecond);
      hoodAppliedVolts = table.get("HoodAppliedVolts", hoodAppliedVolts);
      hoodSupplyCurrentAmps = table.get("HoodSupplyCurrentAmps", hoodSupplyCurrentAmps);
      hoodStatorCurrentAmps = table.get("HoodStatorCurrentAmps", hoodStatorCurrentAmps);
      hoodTemperatureCelsius = table.get("HoodTemperatureCelsius", hoodTemperatureCelsius);

      drumVelocityRpm = table.get("DrumVelocityRpm", drumVelocityRpm);
      drumConnected = table.get("DrumConnected", drumConnected);
      drumAppliedVolts = table.get("DrumAppliedVolts", drumAppliedVolts);
      drumSupplyCurrentAmps = table.get("DrumSupplyCurrentAmps", drumSupplyCurrentAmps);
      drumStatorCurrentAmps = table.get("DrumStatorCurrentAmps", drumStatorCurrentAmps);
      drumTemperatureCelsius = table.get("DrumTemperatureCelsius", drumTemperatureCelsius);
    }
  }

  /** Reads the latest sensor values into {@code inputs}. */
  default void updateInputs(ShooterIOInputs inputs) {}

  /** Sends a Phoenix control request to the hood motor. */
  default void setHoodControl(ControlRequest request) {}

  /** Sends a Phoenix control request to the drum leader; the followers mirror it. */
  default void setDrumControl(ControlRequest request) {}

  /** Seeds the hood encoder with a known mechanism angle, used by homing. */
  default void setHoodPosition(double degrees) {}

  /** Real shooter hardware: a TalonFX hood and a leader + three follower TalonFX drum. */
  class Real implements ShooterIO {
    protected final TalonFX hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_ID);
    protected final TalonFX drumLeader = new TalonFX(ShooterConstants.DRUM_LEADER_ID);
    protected final TalonFX drumFollowerOne = new TalonFX(ShooterConstants.DRUM_FOLLOWER_ONE_ID);
    protected final TalonFX drumFollowerTwo = new TalonFX(ShooterConstants.DRUM_FOLLOWER_TWO_ID);
    protected final TalonFX drumFollowerThree =
        new TalonFX(ShooterConstants.DRUM_FOLLOWER_THREE_ID);

    /** Same order as the drum array inputs. */
    protected final TalonFX[] drumMotors = {
      drumLeader, drumFollowerOne, drumFollowerTwo, drumFollowerThree
    };

    private final StatusSignal<Angle> hoodPosition = hoodMotor.getPosition();
    private final StatusSignal<AngularVelocity> hoodVelocity = hoodMotor.getVelocity();
    private final StatusSignal<Voltage> hoodAppliedVolts = hoodMotor.getMotorVoltage();
    private final StatusSignal<Current> hoodSupplyCurrent = hoodMotor.getSupplyCurrent();
    private final StatusSignal<Current> hoodStatorCurrent = hoodMotor.getStatorCurrent();
    private final StatusSignal<Temperature> hoodTemperature = hoodMotor.getDeviceTemp();

    private final StatusSignal<AngularVelocity> drumVelocity = drumLeader.getVelocity();
    private final List<StatusSignal<Voltage>> drumAppliedVolts = new ArrayList<>();
    private final List<StatusSignal<Current>> drumSupplyCurrent = new ArrayList<>();
    private final List<StatusSignal<Current>> drumStatorCurrent = new ArrayList<>();
    private final List<StatusSignal<Temperature>> drumTemperature = new ArrayList<>();

    private final Debouncer hoodConnectedDebounce = new Debouncer(0.5);
    private final Debouncer[] drumConnectedDebounce = new Debouncer[DRUM_MOTOR_COUNT];

    public Real() {
      PhoenixUtil.tryUntilOk(
          "Shooter hood config", () -> hoodMotor.getConfigurator().apply(ShooterConstants.HOOD_CONFIG));
      for (int i = 0; i < DRUM_MOTOR_COUNT; i++) {
        TalonFX drumMotor = drumMotors[i];
        PhoenixUtil.tryUntilOk(
            "Shooter drum " + i + " config",
            () -> drumMotor.getConfigurator().apply(ShooterConstants.DRUM_CONFIG));
        drumAppliedVolts.add(drumMotor.getMotorVoltage());
        drumSupplyCurrent.add(drumMotor.getSupplyCurrent());
        drumStatorCurrent.add(drumMotor.getStatorCurrent());
        drumTemperature.add(drumMotor.getDeviceTemp());
        drumConnectedDebounce[i] = new Debouncer(0.5);
      }

      // TODO: Verify follower directions on the assembled drum before enabling it.
      drumFollowerOne.setControl(
          new Follower(drumLeader.getDeviceID(), MotorAlignmentValue.Aligned));
      drumFollowerTwo.setControl(
          new Follower(drumLeader.getDeviceID(), MotorAlignmentValue.Opposed));
      drumFollowerThree.setControl(
          new Follower(drumLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
      var hoodStatus =
          BaseStatusSignal.refreshAll(
              hoodPosition,
              hoodVelocity,
              hoodAppliedVolts,
              hoodSupplyCurrent,
              hoodStatorCurrent,
              hoodTemperature);

      inputs.hoodConnected = hoodConnectedDebounce.calculate(hoodStatus.isOK());
      inputs.hoodPositionDegrees = Units.rotationsToDegrees(hoodPosition.getValueAsDouble());
      inputs.hoodVelocityDegreesPerSecond =
          Units.rotationsToDegrees(hoodVelocity.getValueAsDouble());
      inputs.hoodAppliedVolts = hoodAppliedVolts.getValueAsDouble();
      inputs.hoodSupplyCurrentAmps = hoodSupplyCurrent.getValueAsDouble();
      inputs.hoodStatorCurrentAmps = hoodStatorCurrent.getValueAsDouble();
      inputs.hoodTemperatureCelsius = hoodTemperature.getValueAsDouble();

      BaseStatusSignal.refreshAll(drumVelocity);
      inputs.drumVelocityRpm = drumVelocity.getValueAsDouble() * 60.0;

      for (int i = 0; i < DRUM_MOTOR_COUNT; i++) {
        var status =
            BaseStatusSignal.refreshAll(
                drumAppliedVolts.get(i),
                drumSupplyCurrent.get(i),
                drumStatorCurrent.get(i),
                drumTemperature.get(i));
        inputs.drumConnected[i] = drumConnectedDebounce[i].calculate(status.isOK());
        inputs.drumAppliedVolts[i] = drumAppliedVolts.get(i).getValueAsDouble();
        inputs.drumSupplyCurrentAmps[i] = drumSupplyCurrent.get(i).getValueAsDouble();
        inputs.drumStatorCurrentAmps[i] = drumStatorCurrent.get(i).getValueAsDouble();
        inputs.drumTemperatureCelsius[i] = drumTemperature.get(i).getValueAsDouble();
      }
    }

    @Override
    public void setHoodControl(ControlRequest request) {
      hoodMotor.setControl(request);
    }

    @Override
    public void setDrumControl(ControlRequest request) {
      drumLeader.setControl(request);
    }

    @Override
    public void setHoodPosition(double degrees) {
      hoodMotor.setPosition(Units.degreesToRotations(degrees));
    }
  }

  /**
   * Simulated shooter: the TalonFX control loops run in Phoenix sim against DC motor physics. The
   * drum is modeled as all four motors driving one flywheel, stepped through the leader.
   */
  class Sim extends Real {
    private final TalonFXMotorSim hoodSim =
        new TalonFXMotorSim(
            hoodMotor,
            DCMotor.getKrakenX44(1),
            ShooterConstants.HOOD_SIM_MOI_KG_METERS_SQUARED,
            ShooterConstants.HOOD_GEAR_RATIO);
    private final TalonFXMotorSim drumSim =
        new TalonFXMotorSim(
            drumLeader,
            DCMotor.getKrakenX60(DRUM_MOTOR_COUNT),
            ShooterConstants.DRUM_SIM_MOI_KG_METERS_SQUARED,
            ShooterConstants.DRUM_GEAR_RATIO);

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
      hoodSim.update();
      drumSim.update();
      for (int i = 1; i < DRUM_MOTOR_COUNT; i++) {
        drumMotors[i].getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
      }
      super.updateInputs(inputs);
    }
  }
}
