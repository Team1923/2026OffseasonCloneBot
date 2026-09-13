package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.swerve.SwerveIO;
import frc.robot.subsystems.vision.VisionIO;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.Set;
import java.util.TreeSet;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * The IO inputs classes write their own {@code toLog}. This catches a field that was added to an
 * inputs class but never logged, which would silently break logging and replay.
 */
class LoggableInputsTest {
  static Stream<LoggableInputs> allInputs() {
    return Stream.of(
        new IntakeIO.IntakeIOInputs(),
        new RollersIO.RollersIOInputs(),
        new ShooterIO.ShooterIOInputs(),
        new SwerveIO.SwerveIOInputs(),
        new VisionIO.VisionIOInputs());
  }

  @ParameterizedTest
  @MethodSource("allInputs")
  void everyFieldIsLogged(LoggableInputs inputs) {
    Set<String> expectedKeys =
        Arrays.stream(inputs.getClass().getFields())
            .filter(field -> !Modifier.isStatic(field.getModifiers()))
            .map(Field::getName)
            .map(name -> "/" + Character.toUpperCase(name.charAt(0)) + name.substring(1))
            .collect(Collectors.toCollection(TreeSet::new));

    LogTable table = new LogTable(0);
    inputs.toLog(table);
    // Struct types (Pose2d, ...) also write their schema under /.schema, which is not a field
    Set<String> loggedKeys =
        table.getAll(false).keySet().stream()
            .filter(key -> !key.startsWith("/.schema/"))
            .collect(Collectors.toCollection(TreeSet::new));

    assertEquals(expectedKeys, loggedKeys, inputs.getClass().getSimpleName());
  }
}
