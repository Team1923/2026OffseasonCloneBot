package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction for one pose-estimating camera.
 *
 * <p>The default (empty) implementation is used in simulation and during log replay.
 */
public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;

    /** True when the camera produced a MegaTag2 estimate that saw at least one tag. */
    public boolean hasEstimate = false;

    public Pose2d estimatedPose = Pose2d.kZero;
    /** FPGA timestamp of the frame, in seconds. */
    public double timestampSeconds = 0.0;

    public double latencyMilliseconds = 0.0;
    public int tagCount = 0;
    public double tagSpanMeters = 0.0;
    public double averageTagDistanceMeters = 0.0;
    public double averageTagArea = 0.0;
    public int[] tagIds = new int[0];
  }

  /** Reads the latest camera results into {@code inputs}. */
  public default void updateInputs(VisionIOInputs inputs) {}

  /** Sends the robot heading the camera needs to compute a MegaTag2 estimate. */
  public default void setRobotOrientation(double yawDegrees) {}
}
