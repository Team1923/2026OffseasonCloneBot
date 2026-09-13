// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollersConstants;
import frc.robot.StateHandler;
import java.util.function.BooleanSupplier;

/** Controls the hopper floor roller. This will become part of the complete Rollers subsystem. */
public final class RollersSubsystem extends SubsystemBase {
  public enum RollerFloorState {
    REST(rpmRequest(0)),
    INTAKE(rpmRequest(0)), // TODO: measure and tune all these values
    FEED(rpmRequest(0)), 
    EJECT(rpmRequest(0));

    public final VelocityVoltage request;

    RollerFloorState(VelocityVoltage request) {
      this.request = request;
    }

    private static VelocityVoltage rpmRequest(double rpm) {
      return new VelocityVoltage(rpm / 60.0);
    }
  }

  private final StateHandler stateHandler = StateHandler.getInstance();
  private final TalonFX rollerFloorMotor = new TalonFX(RollersConstants.MOTOR_ID);

  public RollersSubsystem() {
    rollerFloorMotor.getConfigurator().apply(RollersConstants.MOTOR_CONFIG);
  }

  public double getSpeedRpm() {
    return rollerFloorMotor.getVelocity().getValueAsDouble() * 60.0;
  }

  /** Applies the desired state. Ordinary commands request states instead of calling this. */
  public void applyState(RollerFloorState desiredState) {
    rollerFloorMotor.setControl(desiredState.request);
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
    return Commands.run(() -> stateHandler.desiredRollerFloorState = desiredState)
        .finallyDo(interrupted -> stateHandler.desiredRollerFloorState = RollerFloorState.REST)
        .withName(name);
  }
}
