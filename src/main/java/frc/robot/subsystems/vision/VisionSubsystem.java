// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateHandler;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.InterpolationConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

/** Adds MegaTag2 pose estimates from all four Limelights to swerve odometry. */
public final class VisionSubsystem extends SubsystemBase {
  private static final String[] CAMERA_NAMES = {"FrontLeft", "FrontRight", "RearLeft", "RearRight"};
  private static final int FRONT_RIGHT = 1;

  private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();

  private final VisionIO[] io;
  private final VisionIO.VisionIOInputs[] inputs;
  private final Alert[] disconnectedAlerts;

  /** Cameras are passed in the order front left, front right, rear left, rear right. */
  public VisionSubsystem(
      VisionIO frontLeft, VisionIO frontRight, VisionIO rearLeft, VisionIO rearRight) {
    io = new VisionIO[] {frontLeft, frontRight, rearLeft, rearRight};
    inputs = new VisionIO.VisionIOInputs[io.length];
    disconnectedAlerts = new Alert[io.length];
    for (int i = 0; i < io.length; i++) {
      inputs[i] = new VisionIO.VisionIOInputs();
      disconnectedAlerts[i] =
          new Alert("Limelight " + CAMERA_NAMES[i] + " disconnected", AlertType.kWarning);
    }
  }

  /** Returns the live drivetrain-estimator pose; Vision does not maintain a second pose cache. */
  public Pose2d getPose() {
    return swerve.getPose();
  }

  /** Returns the alliance-appropriate hub pose used for shot aiming. */
  public Pose2d getAllianceHubPose() {
    return StateHandler.getInstance().isRedAlliance
        ? FieldConstants.hubRedPose
        : FieldConstants.hubBluePose;
  }

  /** Robot-to-hub distance in meters; the key into the shooter interpolation maps. */
  public double getDistanceToHub() {
    return getPose().getTranslation().getDistance(getAllianceHubPose().getTranslation());
  }

  /** Interpolated drum RPM for the current hub distance.*/
  public double getRPM() {
    return InterpolationConstants.distanceToRPMMap.get(getDistanceToHub());
  }

  /** Interpolated hood angle in degrees for the current hub distance. */
  public double getHoodAngle() {
    return InterpolationConstants.distanceToHoodMap.get(getDistanceToHub());
  }

  /** Seeds odometry from the front-right camera when MegaTag1 has a valid target. TODO: Should this MT1 Rezero be done with multiple limelights*/
  public void zeroPoseToMT1() {
    VisionIO.VisionIOInputs frontRight = inputs[FRONT_RIGHT];
    if (!frontRight.hasMegaTag1Estimate) {
      DriverStation.reportWarning("Could not zero pose: front-right Limelight has no target.", false);
      return;
    }
    Logger.recordOutput("Vision/ZeroedPoseToMT1", frontRight.megaTag1Pose);
    swerve.resetPose(frontRight.megaTag1Pose);
  }

  @Override
  public void periodic() {
    // MegaTag2 needs the current robot orientation sent to every Limelight.
    double robotYawDegrees = getPose().getRotation().getDegrees();
    for (int i = 0; i < io.length; i++) {
      io[i].setRobotOrientation(robotYawDegrees);
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/" + CAMERA_NAMES[i], inputs[i]);
      disconnectedAlerts[i].set(!inputs[i].connected);
    }

    // Reject measurements while spinning too quickly for reliable pose estimates.
    boolean spinningTooFast = Math.abs(swerve.getYawVelocityDegreesPerSecond()) >= 720;
    Logger.recordOutput("Vision/RejectedForSpin", spinningTooFast);

    // All four Limelights are intentionally active and independently fused.
    for (int i = 0; i < io.length; i++) {
      addVisionMeasurement(CAMERA_NAMES[i], inputs[i], !spinningTooFast);
    }
  }

  private void addVisionMeasurement(String cameraName, VisionIO.VisionIOInputs camera, boolean allowed) {
    boolean accepted = allowed && camera.hasEstimate;
    Logger.recordOutput("Vision/" + cameraName + "/Accepted", accepted);
    if (!accepted) {
      return;
    }

    double translationStdDev = Math.max(0.05, Math.pow(camera.averageTagDistanceMeters, 3) / 25.0); // TOOD: this needs to be tuned
    Logger.recordOutput("Vision/" + cameraName + "/AcceptedPose", camera.estimatedPose);
    Logger.recordOutput("Vision/" + cameraName + "/TranslationStdDevMeters", translationStdDev);
    swerve.addVisionMeasurement(
        camera.estimatedPose,
        Utils.fpgaToCurrentTime(camera.timestampSeconds),
        VecBuilder.fill(translationStdDev, translationStdDev, 9_999_999));
  }
}
