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

    DesiredStates desired =
        resolve(
            desiredIntakeArmState,
            desiredIntakeRollerState,
            intake.atArmState(desiredIntakeArmState));

    intake.applyArmState(desired.arm());
    intake.applyRollerState(desired.roller());

    if (intake.atArmState(desired.arm())) {
      stateHandler.currentIntakeArmState = desired.arm();
    }
    if (intake.atRollerState(desired.roller())) {
      stateHandler.currentIntakeRollerState = desired.roller();
    }
  }

  /** Pure resolver kept public for focused state-transition tests. */
  public static DesiredStates resolve(
      IntakeArmState desiredArm,
      IntakeRollerState desiredRoller,
      boolean armAtDesiredState) {
    // Do not pull a game piece into an intake that has not finished deploying.
    if (desiredRoller == IntakeRollerState.INTAKE
        && (desiredArm != IntakeArmState.DEPLOYED || !armAtDesiredState)) {
      desiredRoller = IntakeRollerState.REST;
    }

    return new DesiredStates(desiredArm, desiredRoller);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
