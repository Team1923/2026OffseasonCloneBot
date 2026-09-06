package frc.robot;

import frc.robot.subsystems.SwerveSubsystem.SwerveStates;

/**
 * Shared swerve state retained from Rebuilt_2026.
 *
 * <p>Other mechanism states can be added here as the Shooter, Intake, and Rollers
 * state machines are implemented.</p>
 */
public final class StateHandler {
  private static StateHandler stateHandler;

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
