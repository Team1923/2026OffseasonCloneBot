package frc.robot.lib.logging;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.LoggingConstants;
import frc.robot.Constants.LoggingConstants.Mode;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Configures and starts AdvantageKit.
 *
 * <p>On the roboRIO this logs to the USB stick. On the desktop it starts simulation immediately
 * unless AdvantageKit replay settings were supplied through the environment.
 */
public final class LoggingStartup {
  private LoggingStartup() {}

  /**
   * Configures all receivers and starts the logger.
   *
   * @return the mode selected for this process
   */
  public static Mode start(LoggedRobot robot) {
    Logger.recordMetadata("ProjectName", LoggingConstants.PROJECT_NAME);

    Mode mode;
    if (RobotBase.isReal()) {
      Logger.addDataReceiver(new WPILOGWriter(LoggingConstants.REAL_LOG_FOLDER));
      Logger.addDataReceiver(new NT4Publisher());
      mode = Mode.REAL;
    } else if (System.getenv(LoggingConstants.REPLAY_LOG_PATH_ENVIRONMENT_VARIABLE) != null) {
      // Launched by ./gradlew replayWatch or another explicit replay process.
      String logPath = System.getenv(LoggingConstants.REPLAY_LOG_PATH_ENVIRONMENT_VARIABLE);
      boolean realTime =
          Boolean.parseBoolean(
              System.getenv(LoggingConstants.REPLAY_REAL_TIME_ENVIRONMENT_VARIABLE));
      String outputPath = System.getenv(LoggingConstants.REPLAY_OUTPUT_ENVIRONMENT_VARIABLE);
      if (outputPath == null) {
        outputPath = replayOutputNextToSource(logPath);
      } else if (outputPath.equalsIgnoreCase(LoggingConstants.REPLAY_OUTPUT_NONE)) {
        outputPath = null;
      }

      startReplay(robot, logPath, realTime, outputPath);
      mode = Mode.REPLAY;
    } else {
      Logger.addDataReceiver(new WPILOGWriter(LoggingConstants.SIM_LOG_FOLDER));
      Logger.addDataReceiver(new NT4Publisher());
      mode = Mode.SIM;
    }

    Logger.recordMetadata("RobotMode", mode.name());
    Logger.start();
    return mode;
  }

  private static void startReplay(
      LoggedRobot robot, String logPath, boolean realTime, String outputPath) {
    robot.setUseTiming(realTime);
    Logger.setReplaySource(new WPILOGReader(logPath));
    if (outputPath != null) {
      Logger.addDataReceiver(new WPILOGWriter(outputPath));
    }
    Logger.addDataReceiver(new NT4Publisher());
  }

  private static String replayOutputNextToSource(String logPath) {
    return LogFileUtil.addPathSuffix(logPath, LoggingConstants.REPLAY_OUTPUT_SUFFIX);
  }
}
