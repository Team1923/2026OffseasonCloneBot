// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RollersSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/** Multi-subsystem robot actions assembled from subsystem command factories. */
public final class RobotCommands {
  private RobotCommands() {}

  /** Collects game pieces through the intake and into the hopper floor. */
  public static Command collectCommand(
      IntakeSubsystem intake, RollersSubsystem rollers) {
    return Commands.parallel(intake.intakeCommand(), rollers.intakeCommand())
        .withName("CollectGamePieces");
  }

  /**
   * Spins up the shooter, points the drivetrain at the target, and permits feeding only when both
   * mechanisms report ready.
   */
  public static Command shootCommand(
      ShooterSubsystem shooter,
      SwerveSubsystem swerve,
      RollersSubsystem rollers,
      Supplier<Pose2d> targetSupplier) {
    BooleanSupplier feedAllowed =
        () -> shooter.readyToShoot() && swerve.isFacing(VisionSubsystem.getInstance().getAllianceHubPose()); // if isFacing doesn't work can remove

    return Commands.parallel(
            shooter.shootCommand(),
            swerve.faceTargetCommand(targetSupplier),
            rollers.feedWhenAllowedCommand(feedAllowed))
        .withName("ShootGamePieces");
  }
  
  /** Reverses the implemented game-piece path to clear or eject a game piece. */
  public static Command ejectCommand(
      IntakeSubsystem intake, RollersSubsystem rollers) {
    return Commands.parallel(intake.ejectCommand(), rollers.ejectCommand())
        .withName("EjectGamePieces");
  }
}
