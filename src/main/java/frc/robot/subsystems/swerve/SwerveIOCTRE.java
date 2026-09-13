package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.lib.swerve.TunerConstants.TunerSwerveDrivetrain;

/**
 * Swerve IO backed by CTRE's generated drivetrain. {@code setControl}, {@code resetPose},
 * {@code setOperatorPerspectiveForward}, and {@code addVisionMeasurement} are inherited directly
 * from {@code SwerveDrivetrain}.
 */
public class SwerveIOCTRE extends TunerSwerveDrivetrain implements SwerveIO {
  private final StatusSignal<AngularVelocity> gyroYawVelocity;
  private final Debouncer gyroConnectedDebounce = new Debouncer(0.5);

  private final MotorSignals[] driveSignals = new MotorSignals[MODULE_COUNT];
  private final MotorSignals[] steerSignals = new MotorSignals[MODULE_COUNT];
  private final StatusSignal<Angle>[] encoderAbsolutePositions;
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

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  @SuppressWarnings("unchecked")
  public SwerveIOCTRE(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);

    gyroYawVelocity = getPigeon2().getAngularVelocityZWorld();
    encoderAbsolutePositions = new StatusSignal[MODULE_COUNT];
    for (int i = 0; i < MODULE_COUNT; i++) {
      var module = getModule(i);
      driveSignals[i] = new MotorSignals(module.getDriveMotor());
      steerSignals[i] = new MotorSignals(module.getSteerMotor());
      CANcoder encoder = module.getEncoder();
      encoderAbsolutePositions[i] = encoder.getAbsolutePosition();
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
      MotorSignals drive = driveSignals[i];
      inputs.driveConnected[i] = driveConnectedDebounce[i].calculate(drive.refresh());
      inputs.driveAppliedVolts[i] = drive.appliedVolts().getValueAsDouble();
      inputs.driveSupplyCurrentAmps[i] = drive.supplyCurrent().getValueAsDouble();
      inputs.driveStatorCurrentAmps[i] = drive.statorCurrent().getValueAsDouble();
      inputs.driveTemperatureCelsius[i] = drive.temperature().getValueAsDouble();

      MotorSignals steer = steerSignals[i];
      inputs.steerConnected[i] = steerConnectedDebounce[i].calculate(steer.refresh());
      inputs.steerAppliedVolts[i] = steer.appliedVolts().getValueAsDouble();
      inputs.steerSupplyCurrentAmps[i] = steer.supplyCurrent().getValueAsDouble();
      inputs.steerStatorCurrentAmps[i] = steer.statorCurrent().getValueAsDouble();
      inputs.steerTemperatureCelsius[i] = steer.temperature().getValueAsDouble();

      inputs.encoderConnected[i] =
          encoderConnectedDebounce[i].calculate(
              BaseStatusSignal.refreshAll(encoderAbsolutePositions[i]).isOK());
      inputs.encoderAbsolutePositionRotations[i] =
          encoderAbsolutePositions[i].getValueAsDouble();
    }
  }
}
