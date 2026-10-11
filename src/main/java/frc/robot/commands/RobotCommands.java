// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.StateHandler;
import frc.robot.StateHandler.ScoringMode;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.rollers.RollersSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

import java.util.function.BooleanSupplier;
/** Multi-subsystem robot actions assembled from subsystem command factories. */
public final class RobotCommands {
  private RobotCommands() {}

  /** Collects game pieces through the intake and into the hopper floor. */
  public static Command groundIntake(
      IntakeSubsystem intake, RollersSubsystem rollers) {
    return Commands.parallel(intake.intakeCommand(), rollers.intakeCommand())
        .withName("CollectGamePieces");
  }

  /** Creates the complete shooting action for the selected scoring mode. */
  public static Command shootCommand(ScoringMode scoringMode) {
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    RollersSubsystem rollers = RollersSubsystem.getInstance();

    return switch (scoringMode) {
      case RANGED -> rangedShootCommand(shooter, SwerveSubsystem.getInstance(), rollers);
      case FERRY -> ferryShootCommand(shooter, rollers);
    };
  }

  private static Command rangedShootCommand(
      ShooterSubsystem shooter, SwerveSubsystem swerve, RollersSubsystem rollers) {
    BooleanSupplier feedAllowed =
        () ->
            shooter.readyToShoot()
                && swerve.isFacing(VisionSubsystem.getInstance().getAllianceHubPose());

    return Commands.parallel(
            shooter.shootCommand(),
            swerve.faceTargetCommand(() -> VisionSubsystem.getInstance().getAllianceHubPose()),
            rollers.feedWhenAllowedCommand(feedAllowed))
        .withName("ShootGamePieces");
  }

  /**
   * Feeds game pieces between the SOTM and SOTM_END Choreo events. The shooter setpoints are held
   * by {@link #autoShooterCommand()}, which runs for the whole auto routine.
   */
  public static Command shootOnTheMoveCommand() {
    ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    RollersSubsystem rollers = RollersSubsystem.getInstance();
    SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    VisionSubsystem vision = VisionSubsystem.getInstance();

    BooleanSupplier feedAllowed =
        () -> shooter.readyToShoot() && swerve.isFacing(vision.getAllianceHubPose());

    StateHandler stateHandler = StateHandler.getInstance();

    return rollers
        .feedWhenAllowedCommand(feedAllowed)
        .beforeStarting(() -> stateHandler.shootOnTheMoveActive = true)
        .until(() -> !stateHandler.shootOnTheMoveActive || !DriverStation.isAutonomousEnabled())
        .finallyDo(interrupted -> stateHandler.shootOnTheMoveActive = false)
        .onlyIf(DriverStation::isAutonomousEnabled)
        .withName("ShootOnTheMove");
  }

  /**
   * Arms the SOTM shooter setpoints (drum spin-up and hood tracking) without feeding. This is a
   * flag rather than a held command because PathPlanner cancels event commands when a path ends,
   * and the drum has to stay spun up into the next path's SOTM event.
   */
  public static Command prepareShootOnTheMoveCommand() {
    return Commands.runOnce(() -> StateHandler.getInstance().shootOnTheMovePrepared = true)
        .onlyIf(DriverStation::isAutonomousEnabled)
        .withName("PrepareShootOnTheMove");
  }

  /** Ends the SOTM window: stops feeding and lets the drum spin down until the next SOTM_PREP. */
  public static Command stopShootOnTheMoveCommand() {
    return Commands.runOnce(
            () -> {
              StateHandler stateHandler = StateHandler.getInstance();
              stateHandler.shootOnTheMoveActive = false;
              stateHandler.shootOnTheMovePrepared = false;
            })
        .withName("StopShootOnTheMove");
  }

  /**
   * Holds the SOTM shooter setpoints while the SOTM_PREP/SOTM window is open and stows otherwise.
   * Run alongside an entire SOTM auto routine; clears the SOTM flags when the routine ends.
   */
  public static Command autoShooterCommand() {
    StateHandler stateHandler = StateHandler.getInstance();
    return ShooterSubsystem.getInstance()
        .shootOnTheMoveWhenArmedCommand(
            () -> stateHandler.shootOnTheMovePrepared || stateHandler.shootOnTheMoveActive)
        .finallyDo(
            interrupted -> {
              stateHandler.shootOnTheMoveActive = false;
              stateHandler.shootOnTheMovePrepared = false;
            })
        .withName("AutoShooter");
  }

  /** Runs the fixed ferry shot and feeds once the shooter reaches its setpoints. */
  private static Command ferryShootCommand(ShooterSubsystem shooter, RollersSubsystem rollers) {
    return Commands.parallel(
            shooter.ferryCommand(),
            rollers.feedWhenAllowedCommand(shooter::readyToShoot))
        .withName("FerryGamePieces");
  }

  /** Reverses the implemented game-piece path to clear or eject a game piece. */
  public static Command ejectCommand(
      IntakeSubsystem intake, RollersSubsystem rollers) {
    return Commands.parallel(intake.ejectCommand(), rollers.ejectCommand())
        .withName("EjectGamePieces");
  }
}
