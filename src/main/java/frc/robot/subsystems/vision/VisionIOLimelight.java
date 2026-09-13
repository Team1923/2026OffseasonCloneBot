package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.lib.vision.LimelightHelpers;
import frc.robot.lib.vision.LimelightHelpers.PoseEstimate;

/** Vision IO for a Limelight publishing MegaTag2 pose estimates. */
public class VisionIOLimelight implements VisionIO {
  /** A camera that has not published a new frame within this window is reported disconnected. */
  private static final long DISCONNECT_TIMEOUT_MICROSECONDS = 250_000;

  private final String name;
  private final NetworkTableEntry latencyEntry;

  public VisionIOLimelight(String name) {
    this.name = name;
    this.latencyEntry = LimelightHelpers.getLimelightNTTableEntry(name, "tl");
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected =
        RobotController.getFPGATime() - latencyEntry.getLastChange()
            < DISCONNECT_TIMEOUT_MICROSECONDS;

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
