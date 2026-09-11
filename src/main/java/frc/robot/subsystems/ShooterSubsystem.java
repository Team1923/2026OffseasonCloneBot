// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  public static enum ShooterHoodState {
    STOWED(PositionVoltDegrees(0)), // 0 is placeholder, would be the stowed position of the hood
    TRACKING(PositionVoltDegrees(0)), // 0 is placeholder, filled in from table
    FERRY(PositionVoltDegrees(0));  // 0 is placeholder, would be the position of the hood for ferrying FUEL

    public final PositionVoltage request;

    private ShooterHoodState(PositionVoltage request) {
      this.request = request;
    }

    private static PositionVoltage PositionVoltDegrees(double degrees) {
      return new PositionVoltage(Units.degreesToRotations(degrees)); // Convert degrees to rotations for PositionVoltage
    }
  }

  public static enum ShooterDrumState {
    IDLE(VeloVoltRPM(0)),
    WARMING(VeloVoltRPM(0)), // 0 is placeholder, would be slow velocity of the wheels (warming up)
    SHOOTING(VeloVoltRPM(0)), // 0 is placeholder, would be the velocity of the wheels for shooting
    FERRY(VeloVoltRPM(0)); // 0 is placeholder, would be the velocity of the wheels for ferrying FUEL

    public final VelocityVoltage request;

    private ShooterDrumState(VelocityVoltage request) {
      this.request = request;
    }
    
    private static VelocityVoltage VeloVoltRPM(double rpm) {
      return new VelocityVoltage(rpm / 60.0); // Convert RPM to RPS for VelocityVoltage
    }
  }

  private final TalonFX shooterHoodMotor = new TalonFX(Constants.ShooterConstants.shooterHoodID); // Placeholder ID for the hood motor
  private final TalonFX shooterDrumMotor = new TalonFX(Constants.ShooterConstants.shooterDrumID); // Placeholder ID for the drum motor
  private final TalonFX shooterDrumFollowerMotor = new TalonFX(Constants.ShooterConstants.shooterDrumFollowerID); // follower
  private final TalonFX shooterDrumFollowerMotor2 = new TalonFX(Constants.ShooterConstants.shooterDrumFollowerID2); // follower
  private final TalonFX shooterDrumFollowerMotor3 = new TalonFX(Constants.ShooterConstants.shooterDrumFollowerID3); // follower

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // Configure the follower motors to follow the main drum motor
    shooterHoodMotor.getConfigurator().apply(Constants.ShooterConstants.HOOD_CONFIGS);
    shooterDrumMotor.getConfigurator().apply(Constants.ShooterConstants.DRUM_CONFIGS);
    shooterDrumFollowerMotor.setControl(new Follower(shooterDrumMotor.getDeviceID(), MotorAlignmentValue.Aligned));
    shooterDrumFollowerMotor2.setControl(new Follower(shooterDrumMotor.getDeviceID(), MotorAlignmentValue.Opposed)); // not sure about orientation, seems correct
    shooterDrumFollowerMotor3.setControl(new Follower(shooterDrumMotor.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
