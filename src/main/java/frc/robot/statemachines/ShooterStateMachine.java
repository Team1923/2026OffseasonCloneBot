// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statemachines;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.StateHandler;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterDrumState;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterHoodState;
import frc.robot.subsystems.vision.VisionSubsystem;

/** Continuously applies shooter intent and publishes confirmed shooter state. */
public final class ShooterStateMachine extends Command {
  private final ShooterSubsystem shooter;
  private final VisionSubsystem vision;
  private final StateHandler stateHandler = StateHandler.getInstance();

  public ShooterStateMachine(ShooterSubsystem shooter, VisionSubsystem vision) {
    this.shooter = shooter;
    this.vision = vision;
    addRequirements(shooter);
    setName("ShooterStateMachine");
  }

  @Override
  public void execute() {
    ShooterHoodState desiredShooterHoodState = stateHandler.desiredShooterHoodState;
    ShooterDrumState desiredShooterDrumState = stateHandler.desiredShooterDrumState;

    if (desiredShooterHoodState == ShooterHoodState.SHOOTING
        || desiredShooterHoodState == ShooterHoodState.SHOOTING_ON_MOVE) {
      // Keep both stationary and on-the-move targets inside the hood's reachable range.
      double hoodAngleDegrees =
          MathUtil.clamp(
              desiredShooterHoodState == ShooterHoodState.SHOOTING_ON_MOVE
                  ? vision.getShootOnTheMoveHoodAngle()
                  : vision.getHoodAngle(),
              ShooterConstants.HOOD_MIN_DEGREES,
              ShooterConstants.HOOD_MAX_DEGREES);
      desiredShooterHoodState.request.Position = Units.degreesToRotations(hoodAngleDegrees);
    }
    if (desiredShooterDrumState == ShooterDrumState.SHOOTING
        || desiredShooterDrumState == ShooterDrumState.SHOOTING_ON_MOVE) {
      double drumRpm =
          desiredShooterDrumState == ShooterDrumState.SHOOTING_ON_MOVE
              ? vision.getShootOnTheMoveRPM()
              : vision.getRPM();
      desiredShooterDrumState.request.Velocity = drumRpm / 60.0;
    }

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
