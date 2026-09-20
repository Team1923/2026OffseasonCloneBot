// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib.auto.AutoHelpers;

/**
 * Left-side, two-game-piece auto: drives from the trench to the left shot, then runs the
 * second-pass path back out to the trench and back to the same shooting spot. Built from
 * {@code L_TrenchToL_Shoot.traj} and {@code L_Shoot2ndPass.traj} in deploy/choreo.
 */
public class LDoubleNZ extends SequentialCommandGroup {
  public LDoubleNZ() {
    addCommands(
        AutoHelpers.commandPathFromCatchesErrors(AutoHelpers.choreoToPathPlannerConverter("L_TrenchToL_Shoot")),
        new ParallelDeadlineGroup(
          new WaitCommand(5)
          // new ShootCommand() 
        ),
        AutoHelpers.commandPathFromCatchesErrors(AutoHelpers.choreoToPathPlannerConverter("L_Shoot2ndPass")));
  }
}
