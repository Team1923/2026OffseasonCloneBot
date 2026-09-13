package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.ControlRequest;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction for the intake pivot and rollers.
 *
 * <p>The default (empty) implementation is used during log replay, where inputs come from the log.
 */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
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
  }

  /** Reads the latest sensor values into {@code inputs}. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Sends a Phoenix control request to the pivot motor. */
  public default void setArmControl(ControlRequest request) {}

  /** Sends a Phoenix control request to the roller motor. */
  public default void setRollerControl(ControlRequest request) {}

  /** Seeds the pivot encoder with a known mechanism angle. */
  public default void setArmPosition(double degrees) {}
}
