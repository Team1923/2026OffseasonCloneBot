// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.lib.auto.AutoHelpers;

/**
 * Left-side path sequence. Scoring actions will be added when the autonomous command logic is
 * implemented.
 */
public class LDoubleNZ extends SequentialCommandGroup {
  public LDoubleNZ() {
    addCommands(
        follow("SOTM_L_TrenchToL_Shoot"),
        follow("SOTM_L_Shoot2ndPass"),
        follow("SOTM_L_Shoot3rdPass"));
  }

  private static edu.wpi.first.wpilibj2.command.Command follow(String trajectoryName) {
    return AutoHelpers.commandPathFromCatchesErrors(
        AutoHelpers.choreoToPathPlannerConverter(trajectoryName));
  }
}
