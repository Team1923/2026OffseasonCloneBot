// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statemachines;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.subsystems.rollers.RollersSubsystem;
import frc.robot.subsystems.rollers.RollersSubsystem.RollerFloorState;

/** Applies the desired hopper-floor state and publishes confirmed state. */
public final class RollersStateMachine extends Command {
  private final RollersSubsystem rollers;
  private final StateHandler stateHandler = StateHandler.getInstance();

  public RollersStateMachine(RollersSubsystem rollers) {
    this.rollers = rollers;
    addRequirements(rollers);
    setName("RollersStateMachine");
  }

  @Override
  public void execute() {
    RollerFloorState desiredRollerFloorState = stateHandler.desiredRollerFloorState;
    rollers.applyState(desiredRollerFloorState);

    if (rollers.atState(desiredRollerFloorState)) {
      stateHandler.currentRollerFloorState = desiredRollerFloorState;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
