// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.lib.vision.LimelightHelpers;
import frc.robot.lib.vision.LimelightHelpers.PoseEstimate;

/** Adds MegaTag2 pose estimates from all four Limelights to swerve odometry. */
public final class VisionSubsystem extends SubsystemBase {
  private static final double MAX_GYRO_RATE_DEGREES_PER_SECOND = 720.0;
  private static VisionSubsystem visionSubsystem;

  private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();

  private VisionSubsystem() {
    swerve.setVisionMeasurementStdDevs(VecBuilder.fill(0.01, 0.01, 9_999_999));
  }

  public static synchronized VisionSubsystem getVisionSubsystem() {
    if (visionSubsystem == null) {
      visionSubsystem = new VisionSubsystem();
    }
    return visionSubsystem;
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
    if (Math.abs(swerve.getPigeon2().getAngularVelocityZWorld().getValueAsDouble())
        >= MAX_GYRO_RATE_DEGREES_PER_SECOND) {
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
