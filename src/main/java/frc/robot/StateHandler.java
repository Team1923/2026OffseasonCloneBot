package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.IntakeSubsystem.IntakeArmState;
import frc.robot.subsystems.IntakeSubsystem.IntakeRollerState;
import frc.robot.subsystems.RollersSubsystem.RollerFloorState;
import frc.robot.subsystems.ShooterSubsystem.ShooterDrumState;
import frc.robot.subsystems.ShooterSubsystem.ShooterHoodState;
import frc.robot.subsystems.SwerveSubsystem.SwerveStates;

/**
 * Shared robot intent and status.
 *
 * <p>Subsystem command factories are the only writers of desired mechanism states. The matching
 * default state machine is the only writer of each current mechanism state.
 */
public final class StateHandler {
  private static StateHandler stateHandler;

  private StateHandler() {}

  public static synchronized StateHandler getInstance() {
    if (stateHandler == null) {
      stateHandler = new StateHandler();
    }
    return stateHandler;
  }

  public boolean isRedAlliance = false;

  public IntakeArmState desiredIntakeArmState = IntakeArmState.STOWED;
  public IntakeArmState currentIntakeArmState = IntakeArmState.STOWED;
  public IntakeRollerState desiredIntakeRollerState = IntakeRollerState.REST;
  public IntakeRollerState currentIntakeRollerState = IntakeRollerState.REST;

  public RollerFloorState desiredRollerFloorState = RollerFloorState.REST;
  public RollerFloorState currentRollerFloorState = RollerFloorState.REST;

  public ShooterHoodState desiredShooterHoodState = ShooterHoodState.STOWED;
  public ShooterHoodState currentShooterHoodState = ShooterHoodState.STOWED;
  public ShooterDrumState desiredShooterDrumState = ShooterDrumState.IDLE;
  public ShooterDrumState currentShooterDrumState = ShooterDrumState.IDLE;

  public double maxDriveSpeedMetersPerSecond = 3.75;
  public SwerveStates swerveState = SwerveStates.FIELD_CENTRIC;
  public Rotation2d desiredSwerveHeading = Rotation2d.kZero; // this means no rotation
}
