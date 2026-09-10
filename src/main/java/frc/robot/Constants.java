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

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  /** Driver input settings copied from Rebuilt_2026. */
  public static final class ControllerConstants {
    public static final double driverDeadband = 0.01;
  }

    public static final class IntakeConstants {
    public static final int intakeArmID = 0;
    public static final int intakeRollerID = 0;
    public static final double INTAKEARM_GEAR_RATIO = 1;
    public static final double INTAKEROLLERS_GEAR_RATIO = 1; //TODO: get from CAD
    public static final double intakeArmAllowance = 10;


    public static final double intakeArmZeroOffset = -0; // TODO: calibrate
    // need to update sensor ratios
    public static final TalonFXConfiguration INTAKE_ARM_CONFIGS = new TalonFXConfiguration()
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(5)
                .withMotionMagicAcceleration(5)
                .withMotionMagicJerk(0))
        .withSlot0(
            new Slot0Configs()
                .withKP(0).withKI(0).withKD(0).withKV(0).withKS(0)
                .withKG(0)
                .withGravityType(GravityTypeValue.Arm_Cosine))
        .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.Clockwise_Positive))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(90)
                .withSupplyCurrentLimit(40))
        .withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(IntakeConstants.INTAKEARM_GEAR_RATIO))
        .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true).withReverseSoftLimitEnable(true).withForwardSoftLimitThreshold(1).withReverseSoftLimitThreshold(-0.37))
        ;
        

    public static final TalonFXConfiguration INTAKE_ROLLER_CONFIGS = new TalonFXConfiguration()
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicAcceleration(5)
                .withMotionMagicJerk(0))
        .withSlot0(
            new Slot0Configs()
                .withKP(0).withKI(0).withKD(0).withKV(0).withKS(0))
        .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(90)
                .withSupplyCurrentLimit(40))
                
        .withFeedback(
            new FeedbackConfigs()
                .withRotorToSensorRatio(6.4)

        );

  }


  /** NetworkTables names assigned to the robot's four Limelights. */
  public static final class VisionConstants {
    public static final String FRONT_LEFT_LIMELIGHT = "limelight-front-left";
    public static final String FRONT_RIGHT_LIMELIGHT = "limelight-front-right";
    public static final String REAR_LEFT_LIMELIGHT = "limelight-rear-left";
    public static final String REAR_RIGHT_LIMELIGHT = "limelight-rear-right";

  }

  public static final class SwerveConstants {
    public static final double MAX_DRIVE_ANGULAR_VELOCITY = Units.degreesToRadians(720);
    public static final PhoenixPIDController HEADING_CONTROLLER =
        new PhoenixPIDController(3 * Math.PI, 0, 0);
    public static final PathConstraints PATHING_CONSTRAINTS =
        new PathConstraints(2.25, 2.25, Units.degreesToRadians(360),
            Units.degreesToRadians(360));
  }

  public static final class RollerFloorConstants {
    public static final int ROLLERFLOOR_ID = 0;
    public static final double ROLLERFLOOR_RATIO = 0;
    public static final double ROLLERFLOOR_rpmTOLERANCE = 0;

    public static final TalonFXConfiguration ROLLERFLOOR_CONFIG = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(0)
                .withSupplyCurrentLimit(0)
        )
        .withFeedback(
            new FeedbackConfigs()
                .withSensorToMechanismRatio(ROLLERFLOOR_RATIO))
        .withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast))
        .withSlot0(new Slot0Configs()
            .withKP(0)
            .withKV(0)
        );
    }
}