// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.StateHandler;

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

  public double getHoodPosition() {
    return Units.rotationsToDegrees(shooterHoodMotor.getPosition().getValueAsDouble());
  }

  public double getDrumSpeedRPM() {
    return shooterDrumMotor.getVelocity().getValueAsDouble() * 60.0;
  }

  public void setShooterHoodState(ShooterHoodState state) {
    shooterHoodMotor.setControl(state.request);
  }

  public void setShooterDrumState(ShooterDrumState state) {
    shooterDrumMotor.setControl(state.request);
  }

  public boolean isAtShooterHoodState(ShooterHoodState state) {
    return Math.abs(getHoodPosition() - Units.rotationsToDegrees(state.request.Position))
      < Constants.ShooterConstants.hoodAllowanceDegrees;
  }

  public boolean isAtShooterDrumState(ShooterDrumState state) {
    return Math.abs(getDrumSpeedRPM() - state.request.Velocity * 60.0)
      < Constants.ShooterConstants.drumAllowanceRPM;
  }

  public Command aimAtHub() {
    return runEnd(() -> {
      StateHandler.getStateHandler().desiredHoodState = ShooterHoodState.TRACKING;
      StateHandler.getStateHandler().desiredDrumState = ShooterDrumState.WARMING;
    }, () -> {
      StateHandler.getStateHandler().desiredHoodState = ShooterHoodState.STOWED;
      StateHandler.getStateHandler().desiredDrumState = ShooterDrumState.IDLE;
    }).withName("ShooterAimAtHub");
  }

  public Command ferry() {
    return runEnd(() -> {
      StateHandler.getStateHandler().desiredHoodState = ShooterHoodState.FERRY;
      StateHandler.getStateHandler().desiredDrumState = ShooterDrumState.FERRY;
    }, () -> {
      StateHandler.getStateHandler().desiredHoodState = ShooterHoodState.STOWED;
      StateHandler.getStateHandler().desiredDrumState = ShooterDrumState.IDLE;
    }).withName("ShooterFerry");
  }

  public Command rest() {
    return run(() -> {
      StateHandler.getStateHandler().desiredHoodState = ShooterHoodState.STOWED;
      StateHandler.getStateHandler().desiredDrumState = ShooterDrumState.IDLE;
    }).withName("ShooterRest");
  }

  @Override
  public void periodic() {
    setShooterHoodState(StateHandler.getStateHandler().desiredHoodState);
    setShooterDrumState(StateHandler.getStateHandler().desiredDrumState);

    if (isAtShooterHoodState(StateHandler.getStateHandler().desiredHoodState)) {
      StateHandler.getStateHandler().currentHoodState = StateHandler.getStateHandler().desiredHoodState;
    }
    if (isAtShooterDrumState(StateHandler.getStateHandler().desiredDrumState)) {
      StateHandler.getStateHandler().currentDrumState = StateHandler.getStateHandler().desiredDrumState;
    }
  }
}