package frc.robot.lib.phoenix;

import com.ctre.phoenix6.StatusCode;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

/** Helpers for talking to Phoenix 6 devices. */
public final class PhoenixUtil {
  /** Attempts per call; each Phoenix config call already waits up to its own timeout. */
  private static final int MAX_ATTEMPTS = 5;

  private PhoenixUtil() {}

  /**
   * Runs a Phoenix call (config apply, setPosition, ...) until it succeeds, up to five attempts,
   * and reports an error if it never does. A single attempt can fail on a busy bus at boot, which
   * would silently leave the device on its previous or factory configuration.
   *
   * @return whether the call eventually succeeded
   */
  public static boolean tryUntilOk(String description, Supplier<StatusCode> command) {
    StatusCode status = StatusCode.OK;
    for (int attempt = 0; attempt < MAX_ATTEMPTS; attempt++) {
      status = command.get();
      if (status.isOK()) {
        return true;
      }
    }
    DriverStation.reportError(
        description + " failed after " + MAX_ATTEMPTS + " attempts: " + status.getName(), false);
    return false;
  }
}
