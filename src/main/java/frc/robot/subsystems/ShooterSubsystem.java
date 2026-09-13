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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.StateHandler;

/** Controls the four-motor shooter drum and adjustable hood. */
public final class ShooterSubsystem extends SubsystemBase {
  public enum ShooterHoodState {
    STOWED(degreesRequest(0)),
    SHOOTING(degreesRequest(0)), // TODO: replace with shot model
    FERRY(degreesRequest(0)); // TODO: tune

    public final PositionVoltage request;

    ShooterHoodState(PositionVoltage request) {
      this.request = request;
    }

    private static PositionVoltage degreesRequest(double degrees) {
      return new PositionVoltage(Units.degreesToRotations(degrees));
    }
  }

  public enum ShooterDrumState {
    IDLE(rpmRequest(0)),
    WARMING(rpmRequest(0)), // TODO: tune - slow velocity of wheels always running
    SHOOTING(rpmRequest(0)), // TODO: replace with shot model
    FERRY(rpmRequest(0)); // TODO: tune

    public final VelocityVoltage request;

    ShooterDrumState(VelocityVoltage request) {
      this.request = request;
    }

    private static VelocityVoltage rpmRequest(double rpm) {
      return new VelocityVoltage(rpm / 60.0);
    }
  }

  private final StateHandler stateHandler = StateHandler.getInstance();
  private final TalonFX hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_ID);
  private final TalonFX drumLeader = new TalonFX(ShooterConstants.DRUM_LEADER_ID);
  private final TalonFX drumFollowerOne = new TalonFX(ShooterConstants.DRUM_FOLLOWER_ONE_ID);
  private final TalonFX drumFollowerTwo = new TalonFX(ShooterConstants.DRUM_FOLLOWER_TWO_ID);
  private final TalonFX drumFollowerThree = new TalonFX(ShooterConstants.DRUM_FOLLOWER_THREE_ID);

  public ShooterSubsystem() {
    hoodMotor.getConfigurator().apply(ShooterConstants.HOOD_CONFIG);
    drumLeader.getConfigurator().apply(ShooterConstants.DRUM_CONFIG);
    drumFollowerOne.getConfigurator().apply(ShooterConstants.DRUM_CONFIG);
    drumFollowerTwo.getConfigurator().apply(ShooterConstants.DRUM_CONFIG);
    drumFollowerThree.getConfigurator().apply(ShooterConstants.DRUM_CONFIG);

    // TODO: Verify follower directions on the assembled drum before enabling it.
    drumFollowerOne.setControl(
        new Follower(drumLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    drumFollowerTwo.setControl(
        new Follower(drumLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    drumFollowerThree.setControl(
        new Follower(drumLeader.getDeviceID(), MotorAlignmentValue.Opposed));
  }

  public double getHoodPositionDegrees() {
    return Units.rotationsToDegrees(hoodMotor.getPosition().getValueAsDouble());
  }

  public double getDrumSpeedRpm() {
    return drumLeader.getVelocity().getValueAsDouble() * 60.0;
  }

  /** Applies the desired hood state. Ordinary commands request states instead. */
  public void applyHoodState(ShooterHoodState desiredState) {
    hoodMotor.setControl(desiredState.request);
  }

  /** Applies the desired drum state. Ordinary commands request states instead. */
  public void applyDrumState(ShooterDrumState desiredState) {
    drumLeader.setControl(desiredState.request);
  }

  public boolean atHoodState(ShooterHoodState desiredState) {
    return Math.abs(getHoodPositionDegrees() - Units.rotationsToDegrees(desiredState.request.Position))
        < ShooterConstants.HOOD_TOLERANCE_DEGREES;
  }

  public boolean atDrumState(ShooterDrumState desiredState) {
    return Math.abs(getDrumSpeedRpm() - desiredState.request.Velocity * 60.0)
        < ShooterConstants.DRUM_TOLERANCE_RPM;
  }

  public boolean readyToShoot() {
    boolean activeShot =
        stateHandler.desiredShooterDrumState == ShooterDrumState.SHOOTING
            || stateHandler.desiredShooterDrumState == ShooterDrumState.FERRY;
    boolean configuredSetpoint = stateHandler.desiredShooterDrumState.request.Velocity > 0; // don't NEED this but good safety check
    return activeShot
        && configuredSetpoint
        && atHoodState(stateHandler.desiredShooterHoodState)
        && atDrumState(stateHandler.desiredShooterDrumState);
  }

  public Command shootCommand() {
    return requestWhileHeld(
        ShooterHoodState.SHOOTING, ShooterDrumState.SHOOTING, "ShooterShoot");
  }

  public Command warmCommand() {
    return requestWhileHeld(
        ShooterHoodState.STOWED, ShooterDrumState.WARMING, "ShooterWarm");
  }

  public Command ferryCommand() {
    return requestWhileHeld(
        ShooterHoodState.FERRY, ShooterDrumState.FERRY, "ShooterFerry");
  }

  public Command restCommand() {
    return Commands.runOnce(
            () -> request(ShooterHoodState.STOWED, ShooterDrumState.IDLE))
        .withName("ShooterRest");
  }

  private Command requestWhileHeld(
      ShooterHoodState hoodState, ShooterDrumState drumState, String name) {
    return Commands.runEnd(
            () -> request(hoodState, drumState),
            () -> request(ShooterHoodState.STOWED, ShooterDrumState.IDLE))
        .withName(name);
  }

  private void request(ShooterHoodState hoodState, ShooterDrumState drumState) {
    stateHandler.desiredShooterHoodState = hoodState;
    stateHandler.desiredShooterDrumState = drumState;
  }
}
