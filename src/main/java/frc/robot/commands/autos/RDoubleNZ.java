// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RobotCommands;
import frc.robot.lib.auto.AutoHelpers;

/**
 * Right-side shoot-on-the-move auto: three Choreo paths whose SOTM_PREP / SOTM / SOTM_END / INTAKE
 * event markers drive intaking and scoring. The shooter setpoints are held across path boundaries
 * by {@link RobotCommands#autoShooterCommand()}.
 */
public class RDoubleNZ extends SequentialCommandGroup {
  /** First trajectory followed; the auto selector resets odometry to its start pose. */
  public static final String FIRST_TRAJECTORY = "SOTM_R_TrenchToR_Shoot";

  public RDoubleNZ() {
    addCommands(
        Commands.deadline(
            Commands.sequence(
                AutoHelpers.followChoreo(FIRST_TRAJECTORY),
                AutoHelpers.followChoreo("SOTM_R_Shoot2ndPass"),
                AutoHelpers.followChoreo("SOTM_R_Shoot3rdPass")),
            RobotCommands.autoShooterCommand()));
  }
}
