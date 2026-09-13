// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statemachines;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.IntakeArmState;
import frc.robot.subsystems.IntakeSubsystem.IntakeRollerState;

/** Continuously resolves the desired intake states into safe hardware states. */
public final class IntakeStateMachine extends Command {
  public record DesiredStates(IntakeArmState arm, IntakeRollerState roller) {}

  private final IntakeSubsystem intake;
  private final StateHandler stateHandler = StateHandler.getInstance();

  public IntakeStateMachine(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(intake);
    setName("IntakeStateMachine");
  }

  @Override
  public void execute() {
    IntakeArmState desiredIntakeArmState = stateHandler.desiredIntakeArmState;
    IntakeRollerState desiredIntakeRollerState = stateHandler.desiredIntakeRollerState;

    if (stateHandler.desiredIntakeRollerState == IntakeRollerState.INTAKE
        && (stateHandler.currentIntakeArmState != stateHandler.desiredIntakeArmState)) {
          stateHandler.desiredIntakeRollerState = IntakeRollerState.REST;
    }

    intake.applyArmState(desiredIntakeArmState);
    intake.applyRollerState(desiredIntakeRollerState);

    if (intake.atArmState(desiredIntakeArmState)) {
      stateHandler.currentIntakeArmState = desiredIntakeArmState;
    }
    if (intake.atRollerState(desiredIntakeRollerState)) {
      stateHandler.currentIntakeRollerState = desiredIntakeRollerState;
    }
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
