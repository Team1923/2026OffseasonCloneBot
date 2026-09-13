// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statemachines;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.StateHandler;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterDrumState;
import frc.robot.subsystems.ShooterSubsystem.ShooterHoodState;

/** Continuously applies shooter intent and publishes confirmed shooter state. */
public final class ShooterStateMachine extends Command {
  private final ShooterSubsystem shooter;
  private final StateHandler stateHandler = StateHandler.getInstance();

  public ShooterStateMachine(ShooterSubsystem shooter) {
    this.shooter = shooter;
    addRequirements(shooter);
    setName("ShooterStateMachine");
  }

  @Override
  public void execute() {
    ShooterHoodState desiredShooterHoodState = stateHandler.desiredShooterHoodState;
    ShooterDrumState desiredShooterDrumState = stateHandler.desiredShooterDrumState;

    shooter.applyHoodState(desiredShooterHoodState);
    shooter.applyDrumState(desiredShooterDrumState);

    if (shooter.atHoodState(desiredShooterHoodState)) {
      stateHandler.currentShooterHoodState = desiredShooterHoodState;
    }
    if (shooter.atDrumState(desiredShooterDrumState)) {
      stateHandler.currentShooterDrumState = desiredShooterDrumState;
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
