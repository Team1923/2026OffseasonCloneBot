// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.auto;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * Loads a Choreo trajectory (deploy/choreo/&lt;trajectoryName&gt;.traj) and wraps it into a
 * runnable auto Command. Autonomous-period paths in this codebase are authored in Choreo and
 * converted through {@link #choreoToPathPlannerConverter}; PathPlanner's own follower
 * (AutoBuilder) is what actually drives the robot along them.
 */
public final class AutoHelpers {

  private AutoHelpers() {}

  // Reads an existing Choreo .traj file (deploy/choreo/<trajectoryName>.traj) into a
  // PathPlannerPath. Nothing is written to disk here - this just parses the file into memory.
  public static PathPlannerPath choreoToPathPlannerConverter(String trajectoryName) {
    try {
      return PathPlannerPath.fromChoreoTrajectory(trajectoryName);
    } catch (FileVersionException | IOException | ParseException e) {
      DriverStation.reportError(
          "Failed to load Choreo trajectory " + trajectoryName + ": " + e.getMessage(), false);
    } catch (RuntimeException e) {
      // A malformed .traj file (e.g. an event with an unresolved timestamp) can make
      // PathPlannerLib's parser throw an unchecked exception instead of one of the checked
      // ones above. Never let that crash the whole robot program over one bad trajectory.
      DriverStation.reportError("Failed to parse Choreo trajectory " + trajectoryName + ": " + e, false);
    }

    return null;
  }

  // Wraps an already-built PathPlannerPath (e.g. one produced by choreoToPathPlannerConverter)
  // in a Command that follows it via PathPlanner's AutoBuilder. Falls back to a command that
  // finishes instantly if the path failed to load, so one bad trajectory doesn't stall an auto.
  public static Command commandPathFromCatchesErrors(PathPlannerPath path) {
    if (path == null) {
      return generateBlankCommand();
    }
    if (!AutoBuilder.isConfigured()) {
      // AutoBuilder.followPath throws if SwerveSubsystem could not configure AutoBuilder (e.g. a
      // missing deploy/pathplanner/settings.json), and building an auto must never crash the robot.
      DriverStation.reportError(
          "AutoBuilder is not configured; skipping path " + path.name, false);
      return generateBlankCommand();
    }

    return AutoBuilder.followPath(path);
  }

  // Resets odometry to where the path starts (flipped for red), so an auto begins with the right
  // heading even if vision never saw a tag while disabled. Skipped when vision already seeded the
  // pose, since a multi-tag estimate of where the robot really is beats the nominal start pose.
  // Does nothing if the path failed to load.
  public static Command resetPoseToPathStart(PathPlannerPath path) {
    if (path == null || !AutoBuilder.isConfigured()) {
      return generateBlankCommand();
    }

    return path.getStartingHolonomicPose()
        .map(
            pose ->
                Commands.either(
                    Commands.none(),
                    AutoBuilder.resetOdom(pose),
                    () -> VisionSubsystem.getInstance().hasSeededPose()))
        .orElseGet(AutoHelpers::generateBlankCommand);
  }

  public static Command generateBlankCommand() {
    return new Command() {
      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }
}
