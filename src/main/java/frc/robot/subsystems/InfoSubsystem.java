// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.StateHandler;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeArmState;
import frc.robot.subsystems.rollers.RollersSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.util.Optional;

/** Publishes robot state and mechanism telemetry to SmartDashboard. */
public final class InfoSubsystem extends SubsystemBase {
  private final StateHandler stateHandler = StateHandler.getInstance();
  private final IntakeSubsystem intakeSubsystem;
  private final RollersSubsystem rollersSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final SwerveSubsystem swerveSubsystem;
  private final VisionSubsystem visionSubsystem;

  public InfoSubsystem(
      IntakeSubsystem intakeSubsystem,
      RollersSubsystem rollersSubsystem,
      ShooterSubsystem shooterSubsystem,
      SwerveSubsystem swerveSubsystem,
      VisionSubsystem visionSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.rollersSubsystem = rollersSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.swerveSubsystem = swerveSubsystem;
    this.visionSubsystem = visionSubsystem;
  }

  /** Returns whether the alliance hub is active during the current match period. */
  public boolean isHubActive() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return false;
    }
    if (DriverStation.isAutonomousEnabled()) {
      return true;
    }
    if (!DriverStation.isTeleopEnabled()) {
      return false;
    }

    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData.isEmpty()) {
      return true;
    }

    boolean redInactiveFirst;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        return true;
      }
    }

    boolean shiftOneActive =
        switch (alliance.get()) {
          case Red -> !redInactiveFirst;
          case Blue -> redInactiveFirst;
        };

    double matchTime = DriverStation.getMatchTime();
    if (matchTime > 130) {
      return true;
    } else if (matchTime > 105) {
      return shiftOneActive;
    } else if (matchTime > 80) {
      return !shiftOneActive;
    } else if (matchTime > 55) {
      return shiftOneActive;
    } else if (matchTime > 30) {
      return !shiftOneActive;
    }
    return true;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Hub Active", isHubActive());

    SmartDashboard.putBoolean(
        "Intake Down", stateHandler.currentIntakeArmState == IntakeArmState.DEPLOYED);
    SmartDashboard.putString(
        "Desired Intake Arm State", stateHandler.desiredIntakeArmState.toString());
    SmartDashboard.putString(
        "Current Intake Arm State", stateHandler.currentIntakeArmState.toString());
    SmartDashboard.putString(
        "Desired Intake Roller State", stateHandler.desiredIntakeRollerState.toString());
    SmartDashboard.putString(
        "Current Intake Roller State", stateHandler.currentIntakeRollerState.toString());
    SmartDashboard.putNumber("Intake Arm Position (deg)", intakeSubsystem.getArmPositionDegrees());
    SmartDashboard.putNumber("Intake Roller Speed (RPM)", intakeSubsystem.getRollerSpeedRpm());

    SmartDashboard.putString(
        "Desired Floor Roller State", stateHandler.desiredRollerFloorState.toString());
    SmartDashboard.putString(
        "Current Floor Roller State", stateHandler.currentRollerFloorState.toString());
    SmartDashboard.putNumber("Floor Roller Speed (RPM)", rollersSubsystem.getSpeedRpm());

    SmartDashboard.putString(
        "Desired Shooter Hood State", stateHandler.desiredShooterHoodState.toString());
    SmartDashboard.putString(
        "Current Shooter Hood State", stateHandler.currentShooterHoodState.toString());
    SmartDashboard.putString(
        "Desired Shooter Drum State", stateHandler.desiredShooterDrumState.toString());
    SmartDashboard.putString(
        "Current Shooter Drum State", stateHandler.currentShooterDrumState.toString());
    SmartDashboard.putBoolean("Shooter Ready", shooterSubsystem.readyToShoot());
    SmartDashboard.putNumber(
        "Shooter Hood Position (deg)", shooterSubsystem.getHoodPositionDegrees());
    SmartDashboard.putNumber("Shooter Drum Speed (RPM)", shooterSubsystem.getDrumSpeedRpm());

    SmartDashboard.putNumber(
        "Distance To Hub (in)", Units.metersToInches(visionSubsystem.getDistanceToHub()));
    // SmartDashboard.putNumber(
    //     "Target Shooter Speed (RPM)",
    //     stateHandler.desiredShooterDrumState.request.Velocity * 60.0);
    // SmartDashboard.putNumber(
    //     "Target Hood Angle (deg)",
    //     Units.rotationsToDegrees(stateHandler.desiredShooterHoodState.request.Position));

    Pose2d pose = swerveSubsystem.getPose();
    SmartDashboard.putString("Swerve State", stateHandler.swerveState.toString());
    SmartDashboard.putNumber("Robot X (m)", pose.getX());
    SmartDashboard.putNumber("Robot Y (m)", pose.getY());
    SmartDashboard.putNumber("Robot Heading (deg)", pose.getRotation().getDegrees());
    SmartDashboard.putNumber(
        "Desired Robot Heading (deg)", stateHandler.desiredSwerveHeading.getDegrees());
  }
}
