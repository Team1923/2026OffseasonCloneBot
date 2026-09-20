// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.StateHandler;
import org.littletonrobotics.junction.Logger;

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

  private static final String[] DRUM_MOTOR_NAMES = {
    "leader", "follower one", "follower two", "follower three"
  };

  private final StateHandler stateHandler = StateHandler.getInstance();
  private final ShooterIO io;
  private final ShooterIO.ShooterIOInputs inputs = new ShooterIO.ShooterIOInputs();

  private final Alert hoodDisconnectedAlert =
      new Alert("Shooter hood motor disconnected", AlertType.kError);
  private final Alert[] drumDisconnectedAlerts = new Alert[ShooterIO.DRUM_MOTOR_COUNT];

  public ShooterSubsystem(ShooterIO io) {
    this.io = io;
    for (int i = 0; i < ShooterIO.DRUM_MOTOR_COUNT; i++) {
      drumDisconnectedAlerts[i] =
          new Alert(
              "Shooter drum " + DRUM_MOTOR_NAMES[i] + " motor disconnected", AlertType.kError);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    hoodDisconnectedAlert.set(!inputs.hoodConnected);
    for (int i = 0; i < ShooterIO.DRUM_MOTOR_COUNT; i++) {
      drumDisconnectedAlerts[i].set(!inputs.drumConnected[i]);
    }
    Logger.recordOutput("Shooter/ReadyToShoot", readyToShoot());
  }

  public double getHoodPositionDegrees() {
    return inputs.hoodPositionDegrees;
  }

  public double getDrumSpeedRpm() {
    return inputs.drumVelocityRpm;
  }

  /** Applies the desired hood state. Ordinary commands request states instead. */
  public void applyHoodState(ShooterHoodState desiredState) {
    io.setHoodControl(desiredState.request);
    Logger.recordOutput("Shooter/Hood/AppliedState", desiredState);
    Logger.recordOutput(
        "Shooter/Hood/SetpointDegrees", Units.rotationsToDegrees(desiredState.request.Position));
  }

  /** Applies the desired drum state. Ordinary commands request states instead. */
  public void applyDrumState(ShooterDrumState desiredState) {
    io.setDrumControl(desiredState.request);
    Logger.recordOutput("Shooter/Drum/AppliedState", desiredState);
    Logger.recordOutput("Shooter/Drum/SetpointRpm", desiredState.request.Velocity * 60.0);
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
