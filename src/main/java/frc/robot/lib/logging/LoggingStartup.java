package frc.robot.lib.logging;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LoggingConstants;
import frc.robot.Constants.LoggingConstants.Mode;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.Comparator;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * Configures and starts AdvantageKit.
 *
 * <p>On the roboRIO this logs to the USB stick immediately. On the desktop it publishes a "Robot
 * Startup" section to SmartDashboard and waits until Start is pressed, so simulate vs. replay and
 * where logs go are picked from the dashboard instead of in code. Selections are remembered.
 */
public final class LoggingStartup {
  /** What the desktop program should do. */
  public enum RunMode {
    SIMULATE("Simulate"),
    REPLAY_FAST("Replay (as fast as possible)"),
    REPLAY_REAL_TIME("Replay (real time, watch live)");

    final String label;

    RunMode(String label) {
      this.label = label;
    }
  }

  /** Which log to replay. */
  public enum ReplaySource {
    NEWEST_IN_LOGS_FOLDER("Newest log in logs folder"),
    ADVANTAGESCOPE("Log open in AdvantageScope"),
    FILE_PATH("Replay Log Path field");

    final String label;

    ReplaySource(String label) {
      this.label = label;
    }
  }

  /** Where the output log is written. */
  public enum LogOutput {
    DEFAULT("Default (sim: logs folder, replay: next to replayed log)"),
    LOGS_FOLDER("Logs folder"),
    CUSTOM_FOLDER("Log Folder field"),
    NONE("Don't save a log");

    final String label;

    LogOutput(String label) {
      this.label = label;
    }
  }

  private static final String TABLE = LoggingConstants.STARTUP_DASHBOARD_TABLE + "/";
  private static final String RUN_MODE_KEY = TABLE + "Run Mode";
  private static final String REPLAY_SOURCE_KEY = TABLE + "Replay Source";
  private static final String LOG_OUTPUT_KEY = TABLE + "Log Output";
  private static final String REPLAY_PATH_KEY = TABLE + "Replay Log Path";
  private static final String LOG_FOLDER_KEY = TABLE + "Log Folder";
  private static final String START_KEY = TABLE + "Start";
  private static final String STATUS_KEY = TABLE + "Status";

  private LoggingStartup() {}

  /**
   * Adds data receivers (and a replay source if chosen), then starts the logger. Must be called
   * before any subsystem or IO is constructed.
   *
   * @return the mode the robot is running in, which selects the IO implementations
   */
  public static Mode start(LoggedRobot robot) {
    Logger.recordMetadata("ProjectName", LoggingConstants.PROJECT_NAME);

    Mode mode;
    if (RobotBase.isReal()) {
      Logger.addDataReceiver(new WPILOGWriter(LoggingConstants.REAL_LOG_FOLDER));
      Logger.addDataReceiver(new NT4Publisher());
      mode = Mode.REAL;
    } else if (System.getenv(LoggingConstants.REPLAY_LOG_PATH_ENVIRONMENT_VARIABLE) != null) {
      // Launched by ./gradlew replayWatch, or relaunched by this class without the Sim GUI.
      // Both pass the replay settings through the environment.
      String logPath = System.getenv(LoggingConstants.REPLAY_LOG_PATH_ENVIRONMENT_VARIABLE);
      boolean realTime =
          Boolean.parseBoolean(
              System.getenv(LoggingConstants.REPLAY_REAL_TIME_ENVIRONMENT_VARIABLE));
      String output = System.getenv(LoggingConstants.REPLAY_OUTPUT_ENVIRONMENT_VARIABLE);
      String outputPath =
          output == null
              ? replayOutputNextToSource(logPath)
              : output.equals(LoggingConstants.REPLAY_OUTPUT_NONE) ? null : output;
      startReplay(robot, logPath, realTime, outputPath);
      mode = Mode.REPLAY;
    } else {
      mode = waitForDashboardSelection(robot);
    }

    Logger.recordMetadata("RobotMode", mode.name());
    Logger.start();
    return mode;
  }

  private static Mode waitForDashboardSelection(LoggedRobot robot) {
    SendableChooser<RunMode> runModeChooser =
        chooser(RUN_MODE_KEY, RunMode.values(), RunMode.SIMULATE, mode -> mode.label);
    SendableChooser<ReplaySource> replaySourceChooser =
        chooser(
            REPLAY_SOURCE_KEY,
            ReplaySource.values(),
            ReplaySource.NEWEST_IN_LOGS_FOLDER,
            source -> source.label);
    SendableChooser<LogOutput> logOutputChooser =
        chooser(LOG_OUTPUT_KEY, LogOutput.values(), LogOutput.DEFAULT, output -> output.label);

    SmartDashboard.putString(REPLAY_PATH_KEY, Preferences.getString(REPLAY_PATH_KEY, ""));
    SmartDashboard.putString(LOG_FOLDER_KEY, Preferences.getString(LOG_FOLDER_KEY, ""));
    SmartDashboard.putBoolean(START_KEY, false);
    setStatus("Choose options under \"" + LoggingConstants.STARTUP_DASHBOARD_TABLE
        + "\" and press Start");

    while (true) {
      SmartDashboard.updateValues();

      if (SmartDashboard.getBoolean(START_KEY, false)) {
        SmartDashboard.putBoolean(START_KEY, false);

        RunMode runMode = runModeChooser.getSelected();
        ReplaySource replaySource = replaySourceChooser.getSelected();
        LogOutput logOutput = logOutputChooser.getSelected();
        String replayPath = SmartDashboard.getString(REPLAY_PATH_KEY, "").trim();
        String logFolder = SmartDashboard.getString(LOG_FOLDER_KEY, "").trim();

        Runnable saveSelection =
            () -> {
              Preferences.setString(RUN_MODE_KEY, runMode.name());
              Preferences.setString(REPLAY_SOURCE_KEY, replaySource.name());
              Preferences.setString(LOG_OUTPUT_KEY, logOutput.name());
              Preferences.setString(REPLAY_PATH_KEY, replayPath);
              Preferences.setString(LOG_FOLDER_KEY, logFolder);
            };
        Optional<String> error =
            tryStart(
                robot, runMode, replaySource, logOutput, replayPath, logFolder, saveSelection);
        if (error.isEmpty()) {
          setStatus("Running: " + runMode.label);
          SmartDashboard.updateValues();
          return runMode == RunMode.SIMULATE ? Mode.SIM : Mode.REPLAY;
        }
        setStatus("Can't start: " + error.get());
      }

      try {
        Thread.sleep(20);
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
        return Mode.SIM;
      }
    }
  }

  /**
   * Validates the selection and configures the logger. Runs {@code saveSelection} once the
   * selection is valid. Returns an error message on failure.
   */
  private static Optional<String> tryStart(
      LoggedRobot robot,
      RunMode runMode,
      ReplaySource replaySource,
      LogOutput logOutput,
      String replayPath,
      String logFolder,
      Runnable saveSelection) {
    if (logOutput == LogOutput.CUSTOM_FOLDER && logFolder.isEmpty()) {
      return Optional.of("Log Output is \"Log Folder field\" but Log Folder is empty");
    }

    if (runMode == RunMode.SIMULATE) {
      String outputPath =
          switch (logOutput) {
            case DEFAULT, LOGS_FOLDER -> LoggingConstants.SIM_LOG_FOLDER;
            case CUSTOM_FOLDER -> logFolder;
            case NONE -> null;
          };
      saveSelection.run();
      if (outputPath != null) {
        Logger.addDataReceiver(new WPILOGWriter(outputPath));
      }
      Logger.addDataReceiver(new NT4Publisher());
      return Optional.empty();
    }

    Optional<String> sourcePath =
        switch (replaySource) {
          case NEWEST_IN_LOGS_FOLDER -> newestLogIn(LoggingConstants.SIM_LOG_FOLDER);
          case ADVANTAGESCOPE -> logOpenInAdvantageScope();
          case FILE_PATH -> replayPath.isEmpty() ? Optional.empty() : Optional.of(replayPath);
        };
    if (sourcePath.isEmpty()) {
      return Optional.of(
          switch (replaySource) {
            case NEWEST_IN_LOGS_FOLDER -> "no .wpilog files in " + LoggingConstants.SIM_LOG_FOLDER;
            case ADVANTAGESCOPE -> "no log is open in AdvantageScope";
            case FILE_PATH -> "Replay Log Path is empty";
          });
    }
    String logPath = sourcePath.get();
    if (!logPath.endsWith(".wpilog") || !new File(logPath).isFile()) {
      return Optional.of("replay log not found: " + logPath);
    }

    String outputPath =
        switch (logOutput) {
          case DEFAULT -> replayOutputNextToSource(logPath);
          case LOGS_FOLDER -> replayOutputIn(LoggingConstants.SIM_LOG_FOLDER, logPath);
          case CUSTOM_FOLDER -> replayOutputIn(logFolder, logPath);
          case NONE -> null;
        };
    boolean realTime = runMode == RunMode.REPLAY_REAL_TIME;
    saveSelection.run();

    String halSimExtensions =
        System.getenv(LoggingConstants.HAL_SIM_EXTENSIONS_ENVIRONMENT_VARIABLE);
    if (halSimExtensions != null && !halSimExtensions.isEmpty()) {
      // AdvantageKit refuses to replay with the Sim GUI or DS extension loaded, because they would
      // overwrite the replayed Driver Station data. Run the replay in a new process without them.
      return relaunchReplayWithoutSimExtensions(logPath, realTime, outputPath);
    }
    startReplay(robot, logPath, realTime, outputPath);
    return Optional.empty();
  }

  /**
   * Starts this robot program again with no HAL sim extensions and the replay settings in the
   * environment, then exits once that replay finishes. Only returns if the relaunch fails.
   */
  private static Optional<String> relaunchReplayWithoutSimExtensions(
      String logPath, boolean realTime, String outputPath) {
    ProcessBuilder builder =
        new ProcessBuilder(
            Path.of(System.getProperty("java.home"), "bin", "java").toString(),
            "-Djava.library.path=" + System.getProperty("java.library.path"),
            "-cp",
            System.getProperty("java.class.path"),
            LoggingConstants.ROBOT_MAIN_CLASS);
    builder.inheritIO();
    Map<String, String> environment = builder.environment();
    environment.remove(LoggingConstants.HAL_SIM_EXTENSIONS_ENVIRONMENT_VARIABLE);
    environment.put(LoggingConstants.REPLAY_LOG_PATH_ENVIRONMENT_VARIABLE, logPath);
    environment.put(
        LoggingConstants.REPLAY_REAL_TIME_ENVIRONMENT_VARIABLE, Boolean.toString(realTime));
    environment.put(
        LoggingConstants.REPLAY_OUTPUT_ENVIRONMENT_VARIABLE,
        outputPath == null ? LoggingConstants.REPLAY_OUTPUT_NONE : outputPath);

    setStatus(
        "Replaying in a new process without the Sim GUI (AdvantageKit requires this). This"
            + " program exits when the replay finishes.");
    SmartDashboard.updateValues();

    // Free the NetworkTables port so the replay process can publish to AdvantageScope
    NetworkTableInstance.getDefault().stopServer();

    Process replay;
    try {
      replay = builder.start();
    } catch (IOException e) {
      NetworkTableInstance.getDefault().startServer();
      return Optional.of("failed to start the replay process: " + e.getMessage());
    }
    Runtime.getRuntime().addShutdownHook(new Thread(replay::destroy));

    int exitCode;
    try {
      exitCode = replay.waitFor();
    } catch (InterruptedException e) {
      replay.destroy();
      exitCode = 1;
    }
    System.exit(exitCode);
    return Optional.empty();
  }

  private static void startReplay(
      LoggedRobot robot, String logPath, boolean realTime, String outputPath) {
    robot.setUseTiming(realTime);
    Logger.setReplaySource(new WPILOGReader(logPath));
    if (outputPath != null) {
      Logger.addDataReceiver(new WPILOGWriter(outputPath));
    }
    if (realTime) {
      // Lets AdvantageScope connect to the simulator and watch the replay as it runs
      Logger.addDataReceiver(new NT4Publisher());
    }
  }

  private static String replayOutputNextToSource(String logPath) {
    return LogFileUtil.addPathSuffix(logPath, LoggingConstants.REPLAY_OUTPUT_SUFFIX);
  }

  private static String replayOutputIn(String folder, String logPath) {
    return Path.of(folder, new File(replayOutputNextToSource(logPath)).getName()).toString();
  }

  /** Newest .wpilog in {@code folder}, skipping logs that are themselves replay outputs. */
  private static Optional<String> newestLogIn(String folder) {
    File[] logs = new File(folder).listFiles();
    if (logs == null) {
      return Optional.empty();
    }
    String suffix = LoggingConstants.REPLAY_OUTPUT_SUFFIX;
    return Arrays.stream(logs)
        .filter(File::isFile)
        .filter(file -> file.getName().endsWith(".wpilog"))
        .filter(file -> !file.getName().matches(".*" + suffix + "(_[0-9]+)?\\.wpilog"))
        .max(Comparator.comparingLong(File::lastModified))
        .map(File::getPath);
  }

  /** AdvantageScope writes the path of its open log here when it is running. */
  private static Optional<String> logOpenInAdvantageScope() {
    Path pathFile =
        Path.of(
            System.getProperty("java.io.tmpdir"), LoggingConstants.ADVANTAGESCOPE_LOG_PATH_FILE);
    try {
      return Files.readAllLines(pathFile).stream().findFirst().map(String::trim);
    } catch (IOException e) {
      return Optional.empty();
    }
  }

  private static <T extends Enum<T>> SendableChooser<T> chooser(
      String key, T[] options, T fallback, Function<T, String> label) {
    T saved = fallback;
    try {
      saved = Enum.valueOf(fallback.getDeclaringClass(), Preferences.getString(key, fallback.name()));
    } catch (IllegalArgumentException e) {
      // Saved value from an older version of this code; keep the fallback
    }

    SendableChooser<T> chooser = new SendableChooser<>();
    for (T option : options) {
      if (option == saved) {
        chooser.setDefaultOption(label.apply(option), option);
      } else {
        chooser.addOption(label.apply(option), option);
      }
    }
    SmartDashboard.putData(key, chooser);
    return chooser;
  }

  private static void setStatus(String status) {
    SmartDashboard.putString(STATUS_KEY, status);
    System.out.println("[Robot Startup] " + status);
  }
}
