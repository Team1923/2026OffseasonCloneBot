// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollersConstants;
import frc.robot.StateHandler;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

/** Controls the hopper floor roller. This will become part of the complete Rollers subsystem. */
public final class RollersSubsystem extends SubsystemBase {
  private static RollersSubsystem rollersSubsystem;

  public static synchronized RollersSubsystem getInstance() {
    if (rollersSubsystem == null) {
      rollersSubsystem =
          new RollersSubsystem(
              switch (StateHandler.getInstance().robotMode) {
                case REAL -> new RollersIO.Real();
                case SIM -> new RollersIO.Sim();
                case REPLAY -> new RollersIO() {};
              });
    }
    return rollersSubsystem;
  }

  public enum RollerFloorState {
    REST(rpmRequest(0)),
    INTAKE(rpmRequest(0)), // TODO: measure and tune all these values, try ~800 RPM
    FEED(rpmRequest(0)), // TODO: try ~1200 RPM, must outpace what the drum consumes
    EJECT(rpmRequest(0)); // TODO: try ~-1000 RPM

    public final VelocityVoltage request;

    RollerFloorState(VelocityVoltage request) {
      this.request = request;
    }

    private static VelocityVoltage rpmRequest(double rpm) {
      return new VelocityVoltage(rpm / 60.0);
    }
  }

  private final StateHandler stateHandler = StateHandler.getInstance();
  private final RollersIO io;
  private final RollersIO.RollersIOInputs inputs = new RollersIO.RollersIOInputs();

  private final Alert floorDisconnectedAlert =
      new Alert("Roller floor motor disconnected", AlertType.kError);

  private RollersSubsystem(RollersIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Rollers", inputs);

    floorDisconnectedAlert.set(!inputs.floorConnected);
  }

  public double getSpeedRpm() {
    return inputs.floorVelocityRpm;
  }

  /** Applies the desired state. Ordinary commands request states instead of calling this. */
  public void applyState(RollerFloorState desiredState) {
    io.setFloorControl(desiredState.request);
    Logger.recordOutput("Rollers/Floor/AppliedState", desiredState);
    Logger.recordOutput("Rollers/Floor/SetpointRpm", desiredState.request.Velocity * 60.0);
  }

  public boolean atState(RollerFloorState desiredState) {
    return Math.abs(desiredState.request.Velocity * 60.0 - getSpeedRpm())
        < RollersConstants.RPM_TOLERANCE;
  }

  public Command intakeCommand() {
    return requestWhileHeld(RollerFloorState.INTAKE, "RollerFloorIntake");
  }

  public Command feedCommand() {
    return requestWhileHeld(RollerFloorState.FEED, "RollerFloorFeed");
  }

  public Command feedWhenAllowedCommand(BooleanSupplier feedAllowed) {
    return Commands.run(
            () ->
                stateHandler.desiredRollerFloorState =
                    feedAllowed.getAsBoolean()
                        ? RollerFloorState.FEED
                        : RollerFloorState.REST)
        .finallyDo(interrupted -> stateHandler.desiredRollerFloorState = RollerFloorState.REST)
        .withName("RollerFloorFeedWhenAllowed");
  }

  public Command ejectCommand() {
    return requestWhileHeld(RollerFloorState.EJECT, "RollerFloorEject");
  }

  public Command restCommand() {
    return Commands.runOnce(
            () -> stateHandler.desiredRollerFloorState = RollerFloorState.REST)
        .withName("RollerFloorRest");
  }

  private Command requestWhileHeld(RollerFloorState desiredState, String name) {
    return Commands.runEnd(
            () -> stateHandler.desiredRollerFloorState = desiredState,
            () -> stateHandler.desiredRollerFloorState = RollerFloorState.REST)
        .withName(name);
  }
}
