package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.lib.swerve.TunerConstants;
import frc.robot.lib.swerve.TunerConstants.TunerSwerveDrivetrain;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Hardware layer for the swerve drivetrain.
 *
 * <ul>
 *   <li>{@link Real} wraps CTRE's generated drivetrain, which runs odometry and module control on
 *       its own high-frequency thread.
 *   <li>{@link Sim} is the same drivetrain with CTRE's built-in module simulation stepped every 5 ms.
 *   <li>{@code new SwerveIO() {}} does nothing and is used for log replay.
 * </ul>
 */
public interface SwerveIO {
  /** Module order used by every per-module array input: front left, front right, back left, back right. */
  int MODULE_COUNT = 4;

  /** Every value the drivetrain reads. Add new fields to {@code toLog} and {@code fromLog}. */
  class SwerveIOInputs implements LoggableInputs {
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

    @Override
    public void toLog(LogTable table) {
      table.put("Pose", pose);
      table.put("Speeds", speeds);
      table.put("ModuleStates", moduleStates);
      table.put("ModuleTargets", moduleTargets);
      table.put("ModulePositions", modulePositions);
      table.put("RawHeading", rawHeading);
      table.put("OdometryTimestampSeconds", odometryTimestampSeconds);
      table.put("OdometryPeriodSeconds", odometryPeriodSeconds);
      table.put("OdometrySuccessfulDaqs", odometrySuccessfulDaqs);
      table.put("OdometryFailedDaqs", odometryFailedDaqs);

      table.put("GyroConnected", gyroConnected);
      table.put("GyroYawVelocityDegreesPerSecond", gyroYawVelocityDegreesPerSecond);

      table.put("DriveConnected", driveConnected);
      table.put("DriveAppliedVolts", driveAppliedVolts);
      table.put("DriveSupplyCurrentAmps", driveSupplyCurrentAmps);
      table.put("DriveStatorCurrentAmps", driveStatorCurrentAmps);
      table.put("DriveTemperatureCelsius", driveTemperatureCelsius);

      table.put("SteerConnected", steerConnected);
      table.put("SteerAppliedVolts", steerAppliedVolts);
      table.put("SteerSupplyCurrentAmps", steerSupplyCurrentAmps);
      table.put("SteerStatorCurrentAmps", steerStatorCurrentAmps);
      table.put("SteerTemperatureCelsius", steerTemperatureCelsius);

      table.put("EncoderConnected", encoderConnected);
      table.put("EncoderAbsolutePositionRotations", encoderAbsolutePositionRotations);
    }

    @Override
    public void fromLog(LogTable table) {
      pose = table.get("Pose", pose);
      speeds = table.get("Speeds", speeds);
      moduleStates = table.get("ModuleStates", moduleStates);
      moduleTargets = table.get("ModuleTargets", moduleTargets);
      modulePositions = table.get("ModulePositions", modulePositions);
      rawHeading = table.get("RawHeading", rawHeading);
      odometryTimestampSeconds = table.get("OdometryTimestampSeconds", odometryTimestampSeconds);
      odometryPeriodSeconds = table.get("OdometryPeriodSeconds", odometryPeriodSeconds);
      odometrySuccessfulDaqs = table.get("OdometrySuccessfulDaqs", odometrySuccessfulDaqs);
      odometryFailedDaqs = table.get("OdometryFailedDaqs", odometryFailedDaqs);

      gyroConnected = table.get("GyroConnected", gyroConnected);
      gyroYawVelocityDegreesPerSecond =
          table.get("GyroYawVelocityDegreesPerSecond", gyroYawVelocityDegreesPerSecond);

      driveConnected = table.get("DriveConnected", driveConnected);
      driveAppliedVolts = table.get("DriveAppliedVolts", driveAppliedVolts);
      driveSupplyCurrentAmps = table.get("DriveSupplyCurrentAmps", driveSupplyCurrentAmps);
      driveStatorCurrentAmps = table.get("DriveStatorCurrentAmps", driveStatorCurrentAmps);
      driveTemperatureCelsius = table.get("DriveTemperatureCelsius", driveTemperatureCelsius);

      steerConnected = table.get("SteerConnected", steerConnected);
      steerAppliedVolts = table.get("SteerAppliedVolts", steerAppliedVolts);
      steerSupplyCurrentAmps = table.get("SteerSupplyCurrentAmps", steerSupplyCurrentAmps);
      steerStatorCurrentAmps = table.get("SteerStatorCurrentAmps", steerStatorCurrentAmps);
      steerTemperatureCelsius = table.get("SteerTemperatureCelsius", steerTemperatureCelsius);

      encoderConnected = table.get("EncoderConnected", encoderConnected);
      encoderAbsolutePositionRotations =
          table.get("EncoderAbsolutePositionRotations", encoderAbsolutePositionRotations);
    }
  }

  /** Reads the latest drivetrain state into {@code inputs}. */
  default void updateInputs(SwerveIOInputs inputs) {}

  /** Applies a CTRE swerve request to the modules. */
  default void setControl(SwerveRequest request) {}

  /** Resets the pose estimate to {@code pose}. */
  default void resetPose(Pose2d pose) {}

  /** Sets which field direction is "forward" for field-centric requests. */
  default void setOperatorPerspectiveForward(Rotation2d fieldDirection) {}

  /**
   * Fuses a vision pose into the estimator.
   *
   * @param timestampSeconds capture time in CTRE's time base (see {@code Utils.fpgaToCurrentTime})
   */
  default void addVisionMeasurement(
      Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {}

  /**
   * Real drivetrain: CTRE's generated {@code SwerveDrivetrain}. {@code setControl}, {@code
   * resetPose}, {@code setOperatorPerspectiveForward}, and {@code addVisionMeasurement} are
   * inherited directly from it.
   */
  class Real extends TunerSwerveDrivetrain implements SwerveIO {
    private final StatusSignal<AngularVelocity> gyroYawVelocity;
    private final Debouncer gyroConnectedDebounce = new Debouncer(0.5);

    private final List<MotorSignals> driveSignals = new ArrayList<>();
    private final List<MotorSignals> steerSignals = new ArrayList<>();
    private final List<StatusSignal<Angle>> encoderAbsolutePositions = new ArrayList<>();
    private final Debouncer[] driveConnectedDebounce = new Debouncer[MODULE_COUNT];
    private final Debouncer[] steerConnectedDebounce = new Debouncer[MODULE_COUNT];
    private final Debouncer[] encoderConnectedDebounce = new Debouncer[MODULE_COUNT];

    private record MotorSignals(
        StatusSignal<Voltage> appliedVolts,
        StatusSignal<Current> supplyCurrent,
        StatusSignal<Current> statorCurrent,
        StatusSignal<Temperature> temperature) {
      MotorSignals(TalonFX motor) {
        this(
            motor.getMotorVoltage(),
            motor.getSupplyCurrent(),
            motor.getStatorCurrent(),
            motor.getDeviceTemp());
      }

      boolean refresh() {
        return BaseStatusSignal.refreshAll(appliedVolts, supplyCurrent, statorCurrent, temperature)
            .isOK();
      }
    }

    /** Constructs the drivetrain from the Tuner X generated constants. */
    public Real() {
      this(
          TunerConstants.DrivetrainConstants,
          TunerConstants.FrontLeft,
          TunerConstants.FrontRight,
          TunerConstants.BackLeft,
          TunerConstants.BackRight);
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules Constants for each specific module
     */
    public Real(
        SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
      super(drivetrainConstants, modules);

      gyroYawVelocity = getPigeon2().getAngularVelocityZWorld();
      for (int i = 0; i < MODULE_COUNT; i++) {
        var module = getModule(i);
        driveSignals.add(new MotorSignals(module.getDriveMotor()));
        steerSignals.add(new MotorSignals(module.getSteerMotor()));
        encoderAbsolutePositions.add(module.getEncoder().getAbsolutePosition());
        driveConnectedDebounce[i] = new Debouncer(0.5);
        steerConnectedDebounce[i] = new Debouncer(0.5);
        encoderConnectedDebounce[i] = new Debouncer(0.5);
      }
    }

    @Override
    public void updateInputs(SwerveIOInputs inputs) {
      var state = getState();
      inputs.pose = state.Pose;
      inputs.speeds = state.Speeds;
      inputs.moduleStates = state.ModuleStates;
      inputs.moduleTargets = state.ModuleTargets;
      inputs.modulePositions = state.ModulePositions;
      inputs.rawHeading = state.RawHeading;
      inputs.odometryTimestampSeconds = state.Timestamp;
      inputs.odometryPeriodSeconds = state.OdometryPeriod;
      inputs.odometrySuccessfulDaqs = state.SuccessfulDaqs;
      inputs.odometryFailedDaqs = state.FailedDaqs;

      inputs.gyroConnected =
          gyroConnectedDebounce.calculate(BaseStatusSignal.refreshAll(gyroYawVelocity).isOK());
      inputs.gyroYawVelocityDegreesPerSecond = gyroYawVelocity.getValueAsDouble();

      for (int i = 0; i < MODULE_COUNT; i++) {
        MotorSignals drive = driveSignals.get(i);
        inputs.driveConnected[i] = driveConnectedDebounce[i].calculate(drive.refresh());
        inputs.driveAppliedVolts[i] = drive.appliedVolts().getValueAsDouble();
        inputs.driveSupplyCurrentAmps[i] = drive.supplyCurrent().getValueAsDouble();
        inputs.driveStatorCurrentAmps[i] = drive.statorCurrent().getValueAsDouble();
        inputs.driveTemperatureCelsius[i] = drive.temperature().getValueAsDouble();

        MotorSignals steer = steerSignals.get(i);
        inputs.steerConnected[i] = steerConnectedDebounce[i].calculate(steer.refresh());
        inputs.steerAppliedVolts[i] = steer.appliedVolts().getValueAsDouble();
        inputs.steerSupplyCurrentAmps[i] = steer.supplyCurrent().getValueAsDouble();
        inputs.steerStatorCurrentAmps[i] = steer.statorCurrent().getValueAsDouble();
        inputs.steerTemperatureCelsius[i] = steer.temperature().getValueAsDouble();

        StatusSignal<Angle> encoder = encoderAbsolutePositions.get(i);
        inputs.encoderConnected[i] =
            encoderConnectedDebounce[i].calculate(BaseStatusSignal.refreshAll(encoder).isOK());
        inputs.encoderAbsolutePositionRotations[i] = encoder.getValueAsDouble();
      }
    }
  }

  /** Simulated drivetrain: CTRE simulates its own modules, stepped by a fast notifier. */
  class Sim extends Real {
    private static final double SIM_LOOP_PERIOD_SECONDS = 0.005; // 5 ms

    private final Notifier simNotifier;
    private double lastSimTime;

    public Sim() {
      lastSimTime = Utils.getCurrentTimeSeconds();

      /* Run simulation at a faster rate so PID gains behave more reasonably */
      simNotifier =
          new Notifier(
              () -> {
                final double currentTime = Utils.getCurrentTimeSeconds();
                double deltaTime = currentTime - lastSimTime;
                lastSimTime = currentTime;

                updateSimState(deltaTime, RobotController.getBatteryVoltage());
              });
      simNotifier.startPeriodic(SIM_LOOP_PERIOD_SECONDS);
    }
  }
}
