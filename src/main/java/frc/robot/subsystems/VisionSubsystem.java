// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateHandler;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.InterpolationConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.lib.vision.LimelightHelpers;
import frc.robot.lib.vision.LimelightHelpers.PoseEstimate;
import java.util.List;

/** Adds MegaTag2 pose estimates from all four Limelights to swerve odometry. */
public final class VisionSubsystem extends SubsystemBase {
  private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();

  private static VisionSubsystem visionSubsystem;

  public static synchronized VisionSubsystem getInstance() {
    if (visionSubsystem == null) {
      visionSubsystem = new VisionSubsystem();
    }
    return visionSubsystem;
  }

  public VisionSubsystem() {
    swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0.01, 0.01, 9_999_999));
  }

  /**
   * Returns the live drivetrain-estimator pose; Vision does not maintain a second
   * pose cache.
   */
  public Pose2d getPose() {
    return swerve.getState().Pose;
  }

  /** Returns the alliance-appropriate hub pose used for shot aiming. */
  public Pose2d getAllianceHubPose() {
    return StateHandler.getInstance().isRedAlliance
        ? FieldConstants.hubRedPose
        : FieldConstants.hubBluePose;
  }

  /**
   * Robot-to-hub distance in meters; the key into the shooter interpolation maps.
   */
  public double getDistanceToHub() {
    return getPose().getTranslation().getDistance(getAllianceHubPose().getTranslation());
  }

  /** Interpolated drum RPM for the current hub distance. */
  public double getRPM() {
    return InterpolationConstants.distanceToRPMMap.get(getDistanceToHub());
  }

  /** Interpolated hood angle in degrees for the current hub distance. */
  public double getHoodAngle() {
    return InterpolationConstants.distanceToHoodMap.get(getDistanceToHub());
  }

  public PoseEstimate getMT2FrontLeft() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
        VisionConstants.FRONT_LEFT_LIMELIGHT);
  }

  public PoseEstimate getMT2FrontRight() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
        VisionConstants.FRONT_RIGHT_LIMELIGHT);
  }

  public PoseEstimate getMT2RearLeft() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
        VisionConstants.REAR_LEFT_LIMELIGHT);
  }

  public PoseEstimate getMT2RearRight() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
        VisionConstants.REAR_RIGHT_LIMELIGHT);
  }

  public PoseEstimate getMT1FrontLeft() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(
        VisionConstants.FRONT_LEFT_LIMELIGHT);
  }

  public PoseEstimate getMT1FrontRight() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(
        VisionConstants.FRONT_RIGHT_LIMELIGHT);
  }

  public PoseEstimate getMT1RearLeft() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(
        VisionConstants.REAR_LEFT_LIMELIGHT);
  }

  public PoseEstimate getMT1RearRight() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(
        VisionConstants.REAR_RIGHT_LIMELIGHT);
  }

  public void zeroPoseToMT1() {
    PoseEstimate bestEstimate = getBestMt1Estimate();

    if (bestEstimate == null) {
      DriverStation.reportWarning("Could not zero pose: No Limelights have a target.", false);
      return;
    }

    swerve.resetPose(bestEstimate.pose);
  }

  public PoseEstimate getBestMt1Estimate() {
    List<PoseEstimate> estimates = List.of(
        getMT1FrontLeft(),
        getMT1FrontRight(),
        getMT1RearLeft(),
        getMT1RearRight());

    return estimates.stream()
        .filter(estimate -> estimate != null && estimate.tagCount > 0)
        .min((a, b) -> {
          if (a.tagCount > 1 != b.tagCount > 1) {
            if (a.tagCount > 1)
              return -1;
            else
              return 1;
          } else
            return Double.compare(a.avgTagDist, b.avgTagDist);
        })
        .orElse(null);
  }

  @Override
  public void periodic() {
    double robotYawDegrees = swerve.getState().Pose.getRotation().getDegrees();

    // MegaTag2 needs the current robot orientation sent to every Limelight.
    LimelightHelpers.SetRobotOrientation(
        VisionConstants.FRONT_LEFT_LIMELIGHT, robotYawDegrees, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(
        VisionConstants.FRONT_RIGHT_LIMELIGHT, robotYawDegrees, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(
        VisionConstants.REAR_LEFT_LIMELIGHT, robotYawDegrees, 0, 0, 0, 0, 0);
    LimelightHelpers.SetRobotOrientation(
        VisionConstants.REAR_RIGHT_LIMELIGHT, robotYawDegrees, 0, 0, 0, 0, 0);

    // Reject measurements while spinning too quickly for reliable pose estimates.
    if (Math.abs(swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) >= 720) {
      return;
    }

    // All four Limelights are intentionally active and independently fused.
    addVisionMeasurement(getMT2FrontLeft());
    addVisionMeasurement(getMT2FrontRight());
    addVisionMeasurement(getMT2RearLeft());
    addVisionMeasurement(getMT2RearRight());
  }

  private void addVisionMeasurement(PoseEstimate estimate) {
    if (estimate == null || estimate.tagCount == 0) {
      return;
    }

    double translationStdDev = Math.max(0.05, Math.pow(estimate.avgTagDist, 3) / 25.0); // TOOD: this needs to be tuned
    swerve.setVisionMeasurementStdDevs(
        VecBuilder.fill(translationStdDev, translationStdDev, 9_999_999));
    swerve.addVisionMeasurement(
        estimate.pose, Utils.fpgaToCurrentTime(estimate.timestampSeconds));
  }
}
