package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.AutoLog;

/**
 * Hardware abstraction for the swerve drivetrain.
 *
 * <p>The real and simulated implementations wrap CTRE's {@code SwerveDrivetrain}, which runs
 * odometry and module control on its own high-frequency thread. The default (empty) implementation
 * is used during log replay, where the drivetrain state comes from the log.
 */
public interface SwerveIO {
  /** Module order used by every per-module array input: front left, front right, back left, back right. */
  public static final int MODULE_COUNT = 4;

  @AutoLog
  public static class SwerveIOInputs {
    // Drivetrain state reported by CTRE's odometry thread
    public Pose2d pose = Pose2d.kZero;
    public ChassisSpeeds speeds = new ChassisSpeeds();
    public SwerveModuleState[] moduleStates = new SwerveModuleState[0];
    public SwerveModuleState[] moduleTargets = new SwerveModuleState[0];
    public SwerveModulePosition[] modulePositions = new SwerveModulePosition[0];
    public Rotation2d rawHeading = Rotation2d.kZero;
    /** Odometry timestamp in CTRE's time base (seconds). */
    public double odometryTimestampSeconds = 0.0;
    public double odometryPeriodSeconds = 0.0;
    public int odometrySuccessfulDaqs = 0;
    public int odometryFailedDaqs = 0;

    // Gyro
    public boolean gyroConnected = false;
    public double gyroYawVelocityDegreesPerSecond = 0.0;

    // Per-module hardware
    public boolean[] driveConnected = new boolean[MODULE_COUNT];
    public double[] driveAppliedVolts = new double[MODULE_COUNT];
    public double[] driveSupplyCurrentAmps = new double[MODULE_COUNT];
    public double[] driveStatorCurrentAmps = new double[MODULE_COUNT];
    public double[] driveTemperatureCelsius = new double[MODULE_COUNT];

    public boolean[] steerConnected = new boolean[MODULE_COUNT];
    public double[] steerAppliedVolts = new double[MODULE_COUNT];
    public double[] steerSupplyCurrentAmps = new double[MODULE_COUNT];
    public double[] steerStatorCurrentAmps = new double[MODULE_COUNT];
    public double[] steerTemperatureCelsius = new double[MODULE_COUNT];

    public boolean[] encoderConnected = new boolean[MODULE_COUNT];
    public double[] encoderAbsolutePositionRotations = new double[MODULE_COUNT];
  }

  /** Reads the latest drivetrain state into {@code inputs}. */
  public default void updateInputs(SwerveIOInputs inputs) {}

  /** Applies a CTRE swerve request to the modules. */
  public default void setControl(SwerveRequest request) {}

  /** Resets the pose estimate to {@code pose}. */
  public default void resetPose(Pose2d pose) {}

  /** Sets which field direction is "forward" for field-centric requests. */
  public default void setOperatorPerspectiveForward(Rotation2d fieldDirection) {}

  /**
   * Fuses a vision pose into the estimator.
   *
   * @param timestampSeconds capture time in CTRE's time base (see {@code Utils.fpgaToCurrentTime})
   */
  public default void addVisionMeasurement(
      Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {}
}
