// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statemachines;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeArmState;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeRollerState;

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

    // Do not pull a game piece into an intake that has not finished moving. Adjust the local only;
    // the StateHandler request stays as asked (see the architecture playbook).
    if (desiredIntakeRollerState == IntakeRollerState.INTAKE
        && stateHandler.currentIntakeArmState != desiredIntakeArmState) {
      desiredIntakeRollerState = IntakeRollerState.REST;
    }

    // Until the arm is homed its encoder zero is unknown, so position control could drive it into
    // the frame. Leave it unpowered and never report it as having reached a state.
    if (intake.isArmHomed()) {
      intake.applyArmState(desiredIntakeArmState);
    } else {
      intake.stopArm();
    }
    intake.applyRollerState(desiredIntakeRollerState);

    if (intake.isArmHomed() && intake.atArmState(desiredIntakeArmState)) {
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
