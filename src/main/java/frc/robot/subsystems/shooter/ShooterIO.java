package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.ControlRequest;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction for the shooter hood and four-motor drum.
 *
 * <p>The default (empty) implementation is used during log replay, where inputs come from the log.
 */
public interface ShooterIO {
  /** Drum motor order used by every drum array input: leader, follower one, two, three. */
  public static final int DRUM_MOTOR_COUNT = 4;

  @AutoLog
  public static class ShooterIOInputs {
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
  }

  /** Reads the latest sensor values into {@code inputs}. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Sends a Phoenix control request to the hood motor. */
  public default void setHoodControl(ControlRequest request) {}

  /** Sends a Phoenix control request to the drum leader; the followers mirror it. */
  public default void setDrumControl(ControlRequest request) {}
}
