// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.auto;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.autos.LDoubleNZ;
import frc.robot.commands.autos.RDoubleNZ;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public final class AutoInstantiateSelector {
  private enum AutoMode {
    SOTM_L_DOUBLE_NZ("SOTM LDoubleNZ", LDoubleNZ::new, "SOTM_L_TrenchToL_Shoot"),
    SOTM_R_DOUBLE_NZ("SOTM RDoubleNZ", RDoubleNZ::new, "SOTM_R_TrenchToR_Shoot");

    private final String displayName;
    private final Supplier<Command> routine;
    private final String firstPath;

    AutoMode(String displayName, Supplier<Command> routine, String firstPath) {
      this.displayName = displayName;
      this.routine = routine;
      this.firstPath = firstPath;
    }
  }

  private final LoggedDashboardChooser<AutoMode> chooser =
      new LoggedDashboardChooser<>("Auto Mode");
  private AutoMode loadedMode;
  private Alliance loadedAlliance;
  private Command loadedCommand;

  public AutoInstantiateSelector() {
    chooser.addDefaultOption(
        AutoMode.SOTM_L_DOUBLE_NZ.displayName, AutoMode.SOTM_L_DOUBLE_NZ);
    chooser.addOption(AutoMode.SOTM_R_DOUBLE_NZ.displayName, AutoMode.SOTM_R_DOUBLE_NZ);
  }

  /** Reloads the autonomous command when the dashboard selection or alliance changes. */
  public void updateSelectedCommand() {
    AutoMode mode = chooser.get();
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    if (loadedCommand != null && mode == loadedMode && alliance == loadedAlliance) {
      return;
    }

    loadedMode = mode;
    loadedAlliance = alliance;

    if (mode == null) {
      loadedCommand = Commands.print("No auto selected!");
      return;
    }

    PathPlannerPath selectedPath = AutoHelpers.choreoToPathPlannerConverter(mode.firstPath);
    if (selectedPath == null) {
      loadedCommand = Commands.print("Path not found: " + mode.firstPath);
      return;
    }

    PathPlannerPath firstPath =
        alliance == Alliance.Red ? selectedPath.flipPath() : selectedPath;
    Pose2d startPose =
        firstPath
            .getStartingHolonomicPose()
            .orElseGet(
                () ->
                    new Pose2d(
                        firstPath.getPoint(0).position,
                        firstPath.getIdealStartingState().rotation()));

    loadedCommand =
        Commands.sequence(
                SwerveSubsystem.getInstance()
                    .runOnce(() -> SwerveSubsystem.getInstance().resetPose(startPose)),
                mode.routine.get())
            .withName(mode.displayName);
  }

  /** Returns the command prepared while disabled, loading it now only as a fallback. */
  public Command getSelectedCommand() {
    updateSelectedCommand();
    Command selectedCommand = loadedCommand;
    loadedCommand = null;
    return selectedCommand;
  }
}
