package frc.robot;

import frc.robot.subsystems.SwerveSubsystem.SwerveStates;

import frc.robot.subsystems.ShooterSubsystem.ShooterHoodState;
import frc.robot.subsystems.ShooterSubsystem.ShooterDrumState;

/**
 * Shared swerve state retained from Rebuilt_2026.
 *
 * <p>Other mechanism states can be added here as the Shooter, Intake, and Rollers
 * state machines are implemented.</p>
 */
public final class StateHandler {
  private static StateHandler stateHandler;

  public ShooterHoodState desiredHoodState = ShooterHoodState.STOWED;
  public ShooterHoodState currentHoodState = ShooterHoodState.STOWED;
  public ShooterDrumState desiredDrumState = ShooterDrumState.IDLE;
  public ShooterDrumState currentDrumState = ShooterDrumState.IDLE;

  public double maxDriveSpeedMetersPerSecond = 3.75;
  public SwerveStates swerveState = SwerveStates.FIELD_CENTRIC;

  private StateHandler() {}

  public static synchronized StateHandler getStateHandler() {
    if (stateHandler == null) {
      stateHandler = new StateHandler();
    }
    return stateHandler;
  }
}
