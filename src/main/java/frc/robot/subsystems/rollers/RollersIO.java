package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.controls.ControlRequest;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction for the hopper floor roller.
 *
 * <p>The default (empty) implementation is used during log replay, where inputs come from the log.
 */
public interface RollersIO {
  @AutoLog
  public static class RollersIOInputs {
    public boolean floorConnected = false;
    public double floorVelocityRpm = 0.0;
    public double floorAppliedVolts = 0.0;
    public double floorSupplyCurrentAmps = 0.0;
    public double floorStatorCurrentAmps = 0.0;
    public double floorTemperatureCelsius = 0.0;
  }

  /** Reads the latest sensor values into {@code inputs}. */
  public default void updateInputs(RollersIOInputs inputs) {}

  /** Sends a Phoenix control request to the floor roller motor. */
  public default void setFloorControl(ControlRequest request) {}
}
