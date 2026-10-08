// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

/** Robot-wide constants grouped by the subsystem that owns them. */
public final class Constants {
  private Constants() {}

  /** AdvantageKit logging and robot mode. */
  public static final class LoggingConstants {
    /** Selects which IO implementations are constructed and where logs are read/written. */
    public enum Mode {
      /** Running on a real robot. */
      REAL,
      /** Running a physics simulator. */
      SIM,
      /** Replaying inputs from a log file. */
      REPLAY
    }

    /** Recorded as metadata in every log. */
    public static final String PROJECT_NAME = "2026OffseasonCloneBot";

    /** The roboRIO only logs when a USB stick is plugged in. */
    public static final String REAL_LOG_FOLDER = "/U/logs";
    /** Simulation logs, relative to the project folder. Also searched for logs to replay. */
    public static final String SIM_LOG_FOLDER = "logs";
    /** Added to a replayed log's filename for the recomputed output log. */
    public static final String REPLAY_OUTPUT_SUFFIX = "_sim";

    /** Set by ./gradlew replayWatch to replay a log instead of starting simulation. */
    public static final String REPLAY_LOG_PATH_ENVIRONMENT_VARIABLE = "AKIT_LOG_PATH";
    /** Optional: "true" to replay in real time. */
    public static final String REPLAY_REAL_TIME_ENVIRONMENT_VARIABLE = "ROBOT_REPLAY_REAL_TIME";
    /** Optional replay output log path, or {@link #REPLAY_OUTPUT_NONE}. */
    public static final String REPLAY_OUTPUT_ENVIRONMENT_VARIABLE = "ROBOT_REPLAY_OUTPUT";
    public static final String REPLAY_OUTPUT_NONE = "none";

    private LoggingConstants() {}
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    private OperatorConstants() {}
  }

  public static final class ControllerConstants {
    public static final double DRIVER_DEADBAND = 0.01;

    private ControllerConstants() {}
  }

  public static final class IntakeConstants {
    // TODO: placeholder IDs; they only need to be unique so the devices don't collide in sim
    public static final int ARM_MOTOR_ID = 15;
    public static final int ROLLER_MOTOR_ID = 16;

    public static final double ARM_GEAR_RATIO = 1.0;
    public static final double ROLLER_GEAR_RATIO = 1.0; // TODO: get from CAD
    public static final double ARM_POSITION_TOLERANCE_DEGREES = 10.0;
    public static final double ARM_ZERO_OFFSET_DEGREES = 0.0; // TODO: calibrate

    // Simulation-only mechanism inertia. TODO: estimate from CAD
    public static final double ARM_SIM_MOI_KG_METERS_SQUARED = 0.1;
    public static final double ROLLER_SIM_MOI_KG_METERS_SQUARED = 0.001;

    public static final TalonFXConfiguration ARM_CONFIG =
        new TalonFXConfiguration()
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(5)
                    .withMotionMagicAcceleration(5)
                    .withMotionMagicJerk(0))
            .withSlot0(
                new Slot0Configs()
                    .withKP(0)
                    .withKI(0)
                    .withKD(0)
                    .withKV(0)
                    .withKS(0)
                    .withKG(0)
                    .withGravityType(GravityTypeValue.Arm_Cosine))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(90)
                    .withSupplyCurrentLimit(40))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(ARM_GEAR_RATIO))
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withReverseSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(1)
                    .withReverseSoftLimitThreshold(-0.37));

    public static final TalonFXConfiguration ROLLER_CONFIG =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(90)
                    .withSupplyCurrentLimit(40))
            .withFeedback(new FeedbackConfigs().withRotorToSensorRatio(ROLLER_GEAR_RATIO));

    private IntakeConstants() {}
  }

  public static final class RollersConstants {
    public static final int MOTOR_ID = 17; // TODO: placeholder ID, unique so sim devices don't collide
    public static final double GEAR_RATIO = 1.0; // TODO: get from CAD
    public static final double RPM_TOLERANCE = 100.0; // TODO: tune

    // Simulation-only mechanism inertia. TODO: estimate from CAD
    public static final double SIM_MOI_KG_METERS_SQUARED = 0.002;

    public static final TalonFXConfiguration MOTOR_CONFIG =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(0)
                    .withSupplyCurrentLimit(0))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(GEAR_RATIO))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Coast))
            .withSlot0(new Slot0Configs().withKP(0).withKV(0));

    private RollersConstants() {}
  }

  public static final class ShooterConstants {
    // Values carried from the in-progress shooter branch; verify CAN-bus conflicts before use.
    public static final int HOOD_MOTOR_ID = 10;
    public static final int DRUM_LEADER_ID = 11;
    public static final int DRUM_FOLLOWER_ONE_ID = 12;
    public static final int DRUM_FOLLOWER_TWO_ID = 13;
    public static final int DRUM_FOLLOWER_THREE_ID = 14;

    public static final double HOOD_GEAR_RATIO = 1.0; // TODO: get from CAD
    public static final double DRUM_GEAR_RATIO = 1.0; // TODO: get from CAD
    public static final double HOOD_TOLERANCE_DEGREES = 0.5;
    public static final double DRUM_TOLERANCE_RPM = 75.0;

    /** Backing speed used by the authored Choreo SOTM segments. */
    public static final double SOTM_BACKING_SPEED = 0.40;
    /**
     * Maximum backing speed accepted by the SOTM feedforward. This bounds corrections if odometry
     * reports a transient velocity spike.
     */
    public static final double SOTM_MAX_BACKING_SPEED_METERS_PER_SECOND = 0.50;
    /** Added drum RPM per m/s of backing speed. TODO: characterize. */
    public static final double SOTM_RPM_PER_METER_PER_SECOND = 0.0;
    /**
     * Added hood degrees per m/s of backing speed. The mechanism geometry determines the sign.
     * TODO: characterize.
     */
    public static final double SOTM_HOOD_DEGREES_PER_METER_PER_SECOND = 0.0;

    // Simulation-only mechanism inertia. TODO: estimate from CAD
    public static final double HOOD_SIM_MOI_KG_METERS_SQUARED = 0.01;
    public static final double DRUM_SIM_MOI_KG_METERS_SQUARED = 0.01;

    public static final TalonFXConfiguration HOOD_CONFIG =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(40)
                    .withSupplyCurrentLimit(30))
            .withSlot0(
                new Slot0Configs().withKP(0).withKI(0).withKD(0).withKV(0).withKS(0))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(HOOD_GEAR_RATIO))
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withReverseSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(Units.degreesToRotations(36))
                    .withReverseSoftLimitThreshold(Units.degreesToRotations(0)));

    public static final TalonFXConfiguration DRUM_CONFIG =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(120)
                    .withSupplyCurrentLimit(60))
            .withSlot0(
                new Slot0Configs().withKP(0).withKI(0).withKD(0).withKV(0).withKS(0))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Coast)
                    .withInverted(InvertedValue.CounterClockwise_Positive))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(DRUM_GEAR_RATIO));

    private ShooterConstants() {}
  }

  public static final class VisionConstants {
    public static final String FRONT_LEFT_LIMELIGHT = "limelight-front-left";
    public static final String FRONT_RIGHT_LIMELIGHT = "limelight-front-right";
    public static final String REAR_LEFT_LIMELIGHT = "limelight-rear-left";
    public static final String REAR_RIGHT_LIMELIGHT = "limelight-rear-right";

    private VisionConstants() {}
  }

  /**
   * Distance-to-setpoint lookups used to aim the shooter from robot-to-hub distance. Populate the
   * maps once the shot is characterized; {@code Vision} queries them through {@code getRPM()} and
   * {@code getHoodAngle()}.
   */
  public static final class InterpolationConstants {
    public static final InterpolatingDoubleTreeMap distanceToHoodMap = new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap distanceToRPMMap = new InterpolatingDoubleTreeMap();

    static {
      // TODO: add tuned (distanceMeters -> hoodDegrees) and (distanceMeters -> drumRpm) entries.
    }

    private InterpolationConstants() {}
  }

  public static final class FieldConstants {
    public static final double fieldLengthX = 16.540;

    // Hub position (blue origin, WPILib blue-alliance coordinates).
    public static final double hubBlueX = 4.621;
    public static final double hubRedX = fieldLengthX - hubBlueX;
    public static final double hubY = 4.021;

    public static final Pose2d hubBluePose = new Pose2d(hubBlueX, hubY, Rotation2d.fromDegrees(180));
    public static final Pose2d hubRedPose = new Pose2d(hubRedX, hubY, Rotation2d.fromDegrees(0));

    private FieldConstants() {}
  }

  public static final class SwerveConstants {
    public static final double MAX_DRIVE_ANGULAR_VELOCITY = Units.degreesToRadians(720);
    public static final double HEADING_TOLERANCE_DEGREES = 5.0;
    public static final PhoenixPIDController HEADING_CONTROLLER =
        new PhoenixPIDController(3 * Math.PI, 0, 0); // TODO: need to tune
    public static final PathConstraints PATHING_CONSTRAINTS =
        new PathConstraints(
            2.25, 2.25, Units.degreesToRadians(360), Units.degreesToRadians(360));

    private SwerveConstants() {}
  }
}
