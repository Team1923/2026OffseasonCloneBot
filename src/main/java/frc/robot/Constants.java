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

    /** SmartDashboard folder holding the desktop simulate/replay selector. */
    public static final String STARTUP_DASHBOARD_TABLE = "Robot Startup";
    /** Set by ./gradlew replayWatch to replay a log without the dashboard selector. */
    public static final String REPLAY_LOG_PATH_ENVIRONMENT_VARIABLE = "AKIT_LOG_PATH";
    /** "true" to replay in real time; set when the selector relaunches the program for replay. */
    public static final String REPLAY_REAL_TIME_ENVIRONMENT_VARIABLE = "ROBOT_REPLAY_REAL_TIME";
    /** Output log path for a relaunched replay, or {@link #REPLAY_OUTPUT_NONE}. */
    public static final String REPLAY_OUTPUT_ENVIRONMENT_VARIABLE = "ROBOT_REPLAY_OUTPUT";
    public static final String REPLAY_OUTPUT_NONE = "none";
    /** Lists the loaded HAL sim extensions (Sim GUI, DS socket); replay requires it to be unset. */
    public static final String HAL_SIM_EXTENSIONS_ENVIRONMENT_VARIABLE = "HALSIM_EXTENSIONS";
    /** Main class started again when a replay is relaunched without sim extensions. */
    public static final String ROBOT_MAIN_CLASS = "frc.robot.Main";
    /** File in the system temp folder where AdvantageScope writes the path of its open log. */
    public static final String ADVANTAGESCOPE_LOG_PATH_FILE = "akit-log-path.txt";

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

    // Gear ratios are motor rotations per mechanism rotation.
    public static final double ARM_GEAR_RATIO = 20.0;
    public static final double ROLLER_GEAR_RATIO = 2.0;
    // Software limits, in degrees from the boot/stowed zero. Assumes positive = deploying.
    // TODO: confirm the arm moves positive when deploying before enabling.
    public static final double ARM_MIN_DEGREES = 0.0;
    public static final double ARM_MAX_DEGREES = 120.0;
    // Arm angle (from the stowed zero) where the arm is parallel to the ground, sticking out.
    public static final double ARM_LEVEL_DEGREES = 120.0;
    // Arm_Cosine applies kG * cos(position + offset), and Phoenix only accepts offsets of -90 to 90.
    // Holding torque here is -kG * cos(position - level): the minus is because positive (deploying)
    // LOWERS the arm once it is past vertical. Since cos(x - 180) = -cos(x), that equals
    // kG * cos(position + (180 - level)), so the offset is 180 - level with kG kept positive.
    // At 120: offset 60, vertical at 30 degrees gets no kG, and stowed (0, leaning back) is pushed
    // toward deploying. Valid for a level angle between 90 and 270 degrees.
    public static final double ARM_GRAVITY_OFFSET_DEGREES = 180.0 - ARM_LEVEL_DEGREES;
    public static final double ARM_POSITION_TOLERANCE_DEGREES = 3.0;
    public static final double ARM_ZERO_OFFSET_DEGREES = 0.0; // TODO: calibrate

    // Simulation-only mechanism inertia. TODO: estimate from CAD
    public static final double ARM_SIM_MOI_KG_METERS_SQUARED = 0.1;
    public static final double ROLLER_SIM_MOI_KG_METERS_SQUARED = 0.001;

    public static final TalonFXConfiguration ARM_CONFIG =
        new TalonFXConfiguration()
            // Mechanism units: 1 rotation = 360 degrees. Cruise 0.75 rot/s = 270 deg/s (15 motor
            // rot/s at 20:1), reaching cruise in 0.25 s. TODO: tune.
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(0.75)
                    .withMotionMagicAcceleration(3)
                    .withMotionMagicJerk(0))
            // Starting points only, all assume ARM_GEAR_RATIO. Tune on the robot in this order:
            // kG (holds the arm level), kS, then kP. kP 144 = 4 V at 10 degrees of error.
            // kG uses ARM_GRAVITY_OFFSET_DEGREES (see above): tune it with the arm held level.
            // TODO: measure all four.
            .withSlot0(
                new Slot0Configs()
                    .withKP(144)
                    .withKI(0)
                    .withKD(0)
                    .withKV(0)
                    .withKS(0.2)
                    .withKG(0.4)
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withGravityArmPositionOffset(
                        Units.degreesToRotations(ARM_GRAVITY_OFFSET_DEGREES)))
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
                    .withForwardSoftLimitThreshold(Units.degreesToRotations(ARM_MAX_DEGREES))
                    .withReverseSoftLimitThreshold(Units.degreesToRotations(ARM_MIN_DEGREES)));

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
            // SensorToMechanismRatio, not RotorToSensorRatio: the latter only applies to a
            // remote/fused sensor, so it was ignored and velocities were reported in rotor units.
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(ROLLER_GEAR_RATIO));

    private IntakeConstants() {}
  }

  public static final class RollersConstants {
    public static final int MOTOR_ID = 17; // TODO: verify against the robot CAN map
    public static final int FEEDER_MOTOR_ONE_ID = 18; // TODO: verify against the robot CAN map
    public static final int FEEDER_MOTOR_TWO_ID = 19; // TODO: verify against the robot CAN map
    public static final double GEAR_RATIO = 3.0; // motor rotations per roller rotation
    public static final double FEEDER_GEAR_RATIO = 5.0 / 2.0; // motor rotations per feeder rotation
    public static final double RPM_TOLERANCE = 100.0; // TODO: tune

    // Simulation-only mechanism inertia. TODO: estimate from CAD
    public static final double SIM_MOI_KG_METERS_SQUARED = 0.002;

    public static final TalonFXConfiguration MOTOR_CONFIG = motorConfig(GEAR_RATIO);
    // Feeder motors run as followers of the floor roller, so their slot gains and inversion are
    // ignored (direction comes from MotorAlignmentValue in RollersIO); the gear ratio only scales
    // their reported position/velocity.
    public static final TalonFXConfiguration FEEDER_MOTOR_CONFIG = motorConfig(FEEDER_GEAR_RATIO);

    private static TalonFXConfiguration motorConfig(double gearRatio) {
      return new TalonFXConfiguration()
          // Were 0 A with the limits enabled, which allowed no output at all. TODO: confirm.
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withStatorCurrentLimitEnable(true)
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimit(60)
                  .withSupplyCurrentLimit(40))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(gearRatio))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          // Starting points only. Velocity gains are per MECHANISM rot/s, so the per-motor values
          // (kV 0.12 = 12 V / 100 rot/s Kraken X60 free speed, kP 0.10) are scaled by the gear
          // ratio. kS is higher than a flywheel because this drags game pieces.
          // TODO: tune kS, then kV, then kP.
          .withSlot0(
              new Slot0Configs()
                  .withKS(0.25)
                  .withKV(0.12 * gearRatio)
                  .withKP(0.10 * gearRatio));
    }

    private RollersConstants() {}
  }

  public static final class ShooterConstants {
    // Values carried from the in-progress shooter branch; verify CAN-bus conflicts before use.
    public static final int HOOD_MOTOR_ID = 10;
    public static final int DRUM_LEADER_ID = 11;
    public static final int DRUM_FOLLOWER_ONE_ID = 12;
    public static final int DRUM_FOLLOWER_TWO_ID = 13;
    public static final int DRUM_FOLLOWER_THREE_ID = 14;

    // Gear ratios are motor rotations per mechanism rotation.
    public static final double HOOD_GEAR_RATIO = 5.0 / 2.0;
    public static final double DRUM_GEAR_RATIO = 13.0 / 9.0;
    // Software limits. 0 is the homed hard stop.
    public static final double HOOD_MIN_DEGREES = 0.0;
    public static final double HOOD_MAX_DEGREES = 37.0;
    public static final double HOOD_TOLERANCE_DEGREES = 0.5;

    // Hood homing: drive gently into the 0 degree hard stop, then call that position zero.
    public static final double HOOD_HOMING_VOLTS = -1.0;
    public static final double HOOD_HOMING_STOPPED_DEGREES_PER_SECOND = 5.0;
    public static final double HOOD_HOMING_MIN_MOVE_SECONDS = 0.5;
    public static final double HOOD_HOMING_STOPPED_SECONDS = 0.25;
    public static final double HOOD_HOMING_TIMEOUT_SECONDS = 3.0;
    public static final double DRUM_TOLERANCE_RPM = 75.0;

    // Simulation-only mechanism inertia. TODO: estimate from CAD
    public static final double HOOD_SIM_MOI_KG_METERS_SQUARED = 0.01;
    public static final double DRUM_SIM_MOI_KG_METERS_SQUARED = 0.01;

    public static final TalonFXConfiguration HOOD_CONFIG =
        new TalonFXConfiguration()
            // Mechanism units: 1 rotation = 360 degrees, and the hood travels 37 degrees total.
            // Cruise 1 rot/s crosses the full range in about 0.1 s. TODO: tune.
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(1.0)
                    .withMotionMagicAcceleration(10.0)
                    .withMotionMagicJerk(0))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(40)
                    .withSupplyCurrentLimit(30))
            // Starting points only. kP 270 = 1.5 V at 2 degrees of error; position gains are volts
            // per MECHANISM ROTATION, so values in the hundreds are normal. Tune kS first, then kP,
            // then kD only if it oscillates. No kG: add one (with Arm_Cosine and 0 = horizontal)
            // only if the hood falls under gravity when disabled. TODO: tune.
            .withSlot0(
                new Slot0Configs().withKP(270).withKI(0).withKD(0).withKV(0).withKS(0.4))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(HOOD_GEAR_RATIO))
            .withSoftwareLimitSwitch(
                new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withReverseSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(Units.degreesToRotations(HOOD_MAX_DEGREES))
                    .withReverseSoftLimitThreshold(Units.degreesToRotations(HOOD_MIN_DEGREES)));

    public static final TalonFXConfiguration DRUM_CONFIG =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimit(120)
                    .withSupplyCurrentLimit(60))
            // Starting points only. kV 0.115 per motor rot/s is just under the theoretical 12 V /
            // 100 rot/s free speed (Kraken X60) so the loop undershoots rather than oscillates.
            // Velocity gains are per MECHANISM rot/s, so kV and kP are scaled by DRUM_GEAR_RATIO.
            // TODO: tune kS, then kV, then kP.
            .withSlot0(
                new Slot0Configs()
                    .withKP(0.10 * DRUM_GEAR_RATIO)
                    .withKI(0)
                    .withKD(0)
                    .withKV(0.115 * DRUM_GEAR_RATIO)
                    .withKS(0.15))
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
