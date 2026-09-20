// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.lib.auto.AutoHelpers;

/**
 * Right-side, two-game-piece auto: drives from the trench to the right shot, then runs the
 * second-pass path back out to the trench and back to the same shooting spot. Built from
 * {@code R_TrenchToR_Shoot.traj} and {@code R_Trench2ndPass.traj} in deploy/choreo.
 */
public class RDoubleNZ extends SequentialCommandGroup {
  public RDoubleNZ() {
    addCommands(
        AutoHelpers.commandPathFromCatchesErrors(AutoHelpers.choreoToPathPlannerConverter("R_TrenchToR_Shoot")),
          new ParallelDeadlineGroup(
            new WaitCommand(5)
            // new ShootCommand() 
          ),
        AutoHelpers.commandPathFromCatchesErrors(AutoHelpers.choreoToPathPlannerConverter("R_Trench2ndPass"))
        // new ShootCommand()
    );
  }
}
