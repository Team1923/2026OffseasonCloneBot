package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.LoggingConstants.Mode;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeArmState;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeRollerState;
import frc.robot.subsystems.rollers.RollersSubsystem.RollerFloorState;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterDrumState;
import frc.robot.subsystems.shooter.ShooterSubsystem.ShooterHoodState;
import frc.robot.subsystems.swerve.SwerveSubsystem.SwerveStates;
import org.littletonrobotics.junction.Logger;

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

  /** Set once by Robot at startup; selects the IO implementations. */
  public Mode robotMode = Mode.SIM;

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

  /** Records every shared state as an AdvantageKit output. Called once per loop by Robot. */
  public void log() {
    Logger.recordOutput("StateHandler/IsRedAlliance", isRedAlliance);

    Logger.recordOutput("StateHandler/Intake/DesiredArmState", desiredIntakeArmState);
    Logger.recordOutput("StateHandler/Intake/CurrentArmState", currentIntakeArmState);
    Logger.recordOutput("StateHandler/Intake/DesiredRollerState", desiredIntakeRollerState);
    Logger.recordOutput("StateHandler/Intake/CurrentRollerState", currentIntakeRollerState);

    Logger.recordOutput("StateHandler/Rollers/DesiredFloorState", desiredRollerFloorState);
    Logger.recordOutput("StateHandler/Rollers/CurrentFloorState", currentRollerFloorState);

    Logger.recordOutput("StateHandler/Shooter/DesiredHoodState", desiredShooterHoodState);
    Logger.recordOutput("StateHandler/Shooter/CurrentHoodState", currentShooterHoodState);
    Logger.recordOutput("StateHandler/Shooter/DesiredDrumState", desiredShooterDrumState);
    Logger.recordOutput("StateHandler/Shooter/CurrentDrumState", currentShooterDrumState);

    Logger.recordOutput("StateHandler/Swerve/MaxDriveSpeedMetersPerSecond", maxDriveSpeedMetersPerSecond);
    Logger.recordOutput("StateHandler/Swerve/State", swerveState);
    Logger.recordOutput("StateHandler/Swerve/DesiredHeading", desiredSwerveHeading);
  }
}
