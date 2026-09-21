// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RobotCommands;
import frc.robot.lib.auto.AutoHelpers;
import frc.robot.subsystems.rollers.RollersSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Shoot-on-the-move left double-NZ auto using {@code L_TrenchToL_Shoot_SOTM.traj} and
 * {@code L_End_SOTM_2ndPass.traj}.
 *
 * <p>The trajectory's {@code shoot} event marker runs the existing complete shooting command.
 */
public class SOTM_LDoubleNZ extends SequentialCommandGroup {
  public SOTM_LDoubleNZ(
      ShooterSubsystem shooter,
      SwerveSubsystem swerve,
      RollersSubsystem rollers,
      VisionSubsystem vision) {
    NamedCommands.registerCommand(
        "shoot",
        RobotCommands.shootCommand(shooter, swerve, rollers, vision::getAllianceHubPose));

    addCommands(
        AutoHelpers.commandPathFromCatchesErrors(
            AutoHelpers.choreoToPathPlannerConverter("L_TrenchToL_Shoot_SOTM")),
        AutoHelpers.commandPathFromCatchesErrors(
            AutoHelpers.choreoToPathPlannerConverter("L_End_SOTM_2ndPass")),
        AutoHelpers.commandPathFromCatchesErrors(
            AutoHelpers.choreoToPathPlannerConverter("L_End_SOTM_3rdPass")));
  }
}
