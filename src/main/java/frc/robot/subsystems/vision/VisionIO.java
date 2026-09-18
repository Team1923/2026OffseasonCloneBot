package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.lib.vision.LimelightHelpers;
import frc.robot.lib.vision.LimelightHelpers.PoseEstimate;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Hardware layer for one pose-estimating camera.
 *
 * <ul>
 *   <li>{@link Limelight} reads MegaTag2 estimates from a Limelight on the robot.
 *   <li>{@code new VisionIO() {}} does nothing; it is used in simulation (Limelights cannot be
 *       simulated) and for log replay.
 * </ul>
 */
public interface VisionIO {
  /** Every value one camera reports. Add new fields to {@code toLog} and {@code fromLog}. */
  class VisionIOInputs implements LoggableInputs {
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

    /** True when the camera produced a MegaTag1 estimate (used to re-zero odometry). */
    public boolean hasMegaTag1Estimate = false;

    public Pose2d megaTag1Pose = Pose2d.kZero;
    public int megaTag1TagCount = 0;
    public double megaTag1AverageTagDistanceMeters = 0.0;
    /** Ambiguity of the first tag the MegaTag1 estimate saw; 1.0 (worst) when unknown. */
    public double megaTag1Ambiguity = 1.0;

    @Override
    public void toLog(LogTable table) {
      table.put("Connected", connected);
      table.put("HasEstimate", hasEstimate);
      table.put("EstimatedPose", estimatedPose);
      table.put("TimestampSeconds", timestampSeconds);
      table.put("LatencyMilliseconds", latencyMilliseconds);
      table.put("TagCount", tagCount);
      table.put("TagSpanMeters", tagSpanMeters);
      table.put("AverageTagDistanceMeters", averageTagDistanceMeters);
      table.put("AverageTagArea", averageTagArea);
      table.put("TagIds", tagIds);
      table.put("HasMegaTag1Estimate", hasMegaTag1Estimate);
      table.put("MegaTag1Pose", megaTag1Pose);
      table.put("MegaTag1TagCount", megaTag1TagCount);
      table.put("MegaTag1AverageTagDistanceMeters", megaTag1AverageTagDistanceMeters);
      table.put("MegaTag1Ambiguity", megaTag1Ambiguity);
    }

    @Override
    public void fromLog(LogTable table) {
      connected = table.get("Connected", connected);
      hasEstimate = table.get("HasEstimate", hasEstimate);
      estimatedPose = table.get("EstimatedPose", estimatedPose);
      timestampSeconds = table.get("TimestampSeconds", timestampSeconds);
      latencyMilliseconds = table.get("LatencyMilliseconds", latencyMilliseconds);
      tagCount = table.get("TagCount", tagCount);
      tagSpanMeters = table.get("TagSpanMeters", tagSpanMeters);
      averageTagDistanceMeters = table.get("AverageTagDistanceMeters", averageTagDistanceMeters);
      averageTagArea = table.get("AverageTagArea", averageTagArea);
      tagIds = table.get("TagIds", tagIds);
      hasMegaTag1Estimate = table.get("HasMegaTag1Estimate", hasMegaTag1Estimate);
      megaTag1Pose = table.get("MegaTag1Pose", megaTag1Pose);
      megaTag1TagCount = table.get("MegaTag1TagCount", megaTag1TagCount);
      megaTag1AverageTagDistanceMeters =
          table.get("MegaTag1AverageTagDistanceMeters", megaTag1AverageTagDistanceMeters);
      megaTag1Ambiguity = table.get("MegaTag1Ambiguity", megaTag1Ambiguity);
    }
  }

  /** Reads the latest camera results into {@code inputs}. */
  default void updateInputs(VisionIOInputs inputs) {}

  /** Sends the robot heading the camera needs to compute a MegaTag2 estimate. */
  default void setRobotOrientation(double yawDegrees) {}

  /** A Limelight publishing MegaTag2 pose estimates over NetworkTables. */
  class Limelight implements VisionIO {
    /** A camera that has not published a new frame within this window is reported disconnected. */
    private static final long DISCONNECT_TIMEOUT_MICROSECONDS = 250_000;

    private final String name;
    private final NetworkTableEntry latencyEntry;

    public Limelight(String name) {
      this.name = name;
      this.latencyEntry = LimelightHelpers.getLimelightNTTableEntry(name, "tl");
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
      inputs.connected =
          RobotController.getFPGATime() - latencyEntry.getLastChange()
              < DISCONNECT_TIMEOUT_MICROSECONDS;

      PoseEstimate megaTag1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
      inputs.hasMegaTag1Estimate = megaTag1 != null && megaTag1.tagCount > 0;
      if (inputs.hasMegaTag1Estimate) {
        inputs.megaTag1Pose = megaTag1.pose;
        inputs.megaTag1TagCount = megaTag1.tagCount;
        inputs.megaTag1AverageTagDistanceMeters = megaTag1.avgTagDist;
        inputs.megaTag1Ambiguity =
            megaTag1.rawFiducials != null && megaTag1.rawFiducials.length > 0
                ? megaTag1.rawFiducials[0].ambiguity
                : 1.0;
      } else {
        inputs.megaTag1TagCount = 0;
      }

      PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
      inputs.hasEstimate = estimate != null && estimate.tagCount > 0;
      if (!inputs.hasEstimate) {
        inputs.tagCount = 0;
        inputs.tagIds = new int[0];
        return;
      }

      inputs.estimatedPose = estimate.pose;
      inputs.timestampSeconds = estimate.timestampSeconds;
      inputs.latencyMilliseconds = estimate.latency;
      inputs.tagCount = estimate.tagCount;
      inputs.tagSpanMeters = estimate.tagSpan;
      inputs.averageTagDistanceMeters = estimate.avgTagDist;
      inputs.averageTagArea = estimate.avgTagArea;

      int[] tagIds = new int[estimate.rawFiducials == null ? 0 : estimate.rawFiducials.length];
      for (int i = 0; i < tagIds.length; i++) {
        tagIds[i] = estimate.rawFiducials[i].id;
      }
      inputs.tagIds = tagIds;
    }

    @Override
    public void setRobotOrientation(double yawDegrees) {
      LimelightHelpers.SetRobotOrientation(name, yawDegrees, 0, 0, 0, 0, 0);
    }
  }
}
