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
import frc.robot.Constants.LoggingConstants.Mode;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

/** Adds MegaTag2 pose estimates from all four Limelights to swerve odometry. */
public final class VisionSubsystem extends SubsystemBase {
  private static final String[] CAMERA_NAMES = {"FrontLeft", "FrontRight", "RearLeft", "RearRight"};

  private static VisionSubsystem visionSubsystem;

  /** Creates the vision subsystem with the Limelight IO on the robot, or empty IO in sim/replay. */
  public static synchronized VisionSubsystem getInstance() {
    if (visionSubsystem == null) {
      visionSubsystem =
          StateHandler.getInstance().robotMode == Mode.REAL
              ? new VisionSubsystem(
                  new VisionIO.Limelight(VisionConstants.FRONT_LEFT_LIMELIGHT),
                  new VisionIO.Limelight(VisionConstants.FRONT_RIGHT_LIMELIGHT),
                  new VisionIO.Limelight(VisionConstants.REAR_LEFT_LIMELIGHT),
                  new VisionIO.Limelight(VisionConstants.REAR_RIGHT_LIMELIGHT))
              // Limelights are not simulated, and in replay every input comes from the log.
              : new VisionSubsystem(
                  new VisionIO() {}, new VisionIO() {}, new VisionIO() {}, new VisionIO() {});
    }
    return visionSubsystem;
  }

  private final SwerveSubsystem swerve = SwerveSubsystem.getInstance();

  private final VisionIO[] io;
  private final VisionIO.VisionIOInputs[] inputs;
  private final Alert[] disconnectedAlerts;
  /** Timestamp of the last MegaTag2 frame seen from each camera, so each frame is fused once. */
  private final double[] lastFrameTimestamps;

  /** Cameras are passed in the order front left, front right, rear left, rear right. */
  private VisionSubsystem(
      VisionIO frontLeft, VisionIO frontRight, VisionIO rearLeft, VisionIO rearRight) {
    io = new VisionIO[] {frontLeft, frontRight, rearLeft, rearRight};
    inputs = new VisionIO.VisionIOInputs[io.length];
    disconnectedAlerts = new Alert[io.length];
    lastFrameTimestamps = new double[io.length];
    Arrays.fill(lastFrameTimestamps, Double.NaN);
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

  /** Interpolated drum RPM for the current hub distance; 0 until the map has entries. */
  public double getRPM() {
    Double rpm = InterpolationConstants.distanceToRPMMap.get(getDistanceToHub());
    return rpm == null ? 0.0 : rpm;
  }

  /** Interpolated hood angle in degrees for the current hub distance; 0 until the map has entries. */
  public double getHoodAngle() {
    Double hoodAngleDegrees = InterpolationConstants.distanceToHoodMap.get(getDistanceToHub());
    return hoodAngleDegrees == null ? 0.0 : hoodAngleDegrees;
  }

  /** Resets odometry to the most trustworthy MegaTag1 estimate from any of the four Limelights. */
  public void zeroPoseToMT1() {
    int best = getBestMt1Camera();

    if (best < 0) {
      DriverStation.reportWarning("Could not zero pose: No Limelights have a target.", false);
      return;
    }

    Logger.recordOutput("Vision/ZeroPoseToMT1/Camera", CAMERA_NAMES[best]);
    Logger.recordOutput("Vision/ZeroPoseToMT1/Pose", inputs[best].megaTag1Pose);
    swerve.resetPose(inputs[best].megaTag1Pose);
  }

  /**
   * Index of the camera with the best MegaTag1 estimate, or -1 if none see a tag. Lower quality
   * tier wins; ties go to the closer average tag distance.
   */
  public int getBestMt1Camera() {
    int best = -1;
    for (int i = 0; i < inputs.length; i++) {
      if (!inputs[i].hasMegaTag1Estimate) {
        continue;
      }
      if (best < 0 || compareMt1Quality(inputs[i], inputs[best]) < 0) {
        best = i;
      }
    }
    return best;
  }

  private static int compareMt1Quality(VisionIO.VisionIOInputs a, VisionIO.VisionIOInputs b) {
    int tierA = getQualityTier(a);
    int tierB = getQualityTier(b);
    if (tierA != tierB) {
      return Integer.compare(tierA, tierB);
    }
    return Double.compare(a.megaTag1AverageTagDistanceMeters, b.megaTag1AverageTagDistanceMeters);
  }

  private static int getQualityTier(VisionIO.VisionIOInputs camera) {
    if (camera.megaTag1TagCount >= 2) {
      return 1;
    } else if (camera.megaTag1Ambiguity <= 0.3) {
      return 2;
    } else {
      return 3;
    }
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
      addVisionMeasurement(i, !spinningTooFast);
    }
  }

  private void addVisionMeasurement(int cameraIndex, boolean allowed) {
    String cameraName = CAMERA_NAMES[cameraIndex];
    VisionIO.VisionIOInputs camera = inputs[cameraIndex];

    // Limelights keep publishing their last estimate, so the same frame shows up every loop until
    // a new one arrives, and forever after a camera drops off. Fuse each frame once, and only from
    // a camera that is still updating.
    boolean newFrame =
        camera.hasEstimate && camera.timestampSeconds != lastFrameTimestamps[cameraIndex];
    lastFrameTimestamps[cameraIndex] = camera.timestampSeconds;

    boolean accepted = allowed && camera.connected && newFrame;
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
