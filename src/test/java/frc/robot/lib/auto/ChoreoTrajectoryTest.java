package frc.robot.lib.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.wpilibj2.command.Commands;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

class ChoreoTrajectoryTest {
  @BeforeAll
  static void registerNamedCommands() {
    NamedCommands.registerCommand("SOTM_PREP", Commands.none());
    NamedCommands.registerCommand("SOTM", Commands.none());
    NamedCommands.registerCommand("SOTM_END", Commands.none());
    NamedCommands.registerCommand("INTAKE", Commands.none());
  }

  @Test
  void pathPlannerRobotConfigLoads() throws Exception {
    assertNotNull(RobotConfig.fromGUISettings());
  }

  @ParameterizedTest
  @ValueSource(
      strings = {
        "SOTM_L_TrenchToL_Shoot",
        "SOTM_L_Shoot2ndPass",
        "SOTM_L_Shoot3rdPass",
        "SOTM_R_TrenchToR_Shoot",
        "SOTM_R_Shoot2ndPass",
        "SOTM_R_Shoot3rdPass"
      })
  void choreoTrajectoryLoads(String trajectoryName) {
    assertNotNull(AutoHelpers.choreoToPathPlannerConverter(trajectoryName));
  }

  @ParameterizedTest
  @ValueSource(strings = {"SOTM_L_TrenchToL_Shoot", "SOTM_R_TrenchToR_Shoot"})
  void initialPathPreparesShooterAndRunsIntake(String trajectoryName) throws Exception {
    JSONObject trajectory = readTrajectory(trajectoryName);
    List<String> eventNames = getEventNames(trajectory);
    assertTrue(eventNames.contains("SOTM_PREP"));
    assertTrue(eventNames.contains("INTAKE"));
    assertEquals(0.0, getEventTimestamp(trajectory, "SOTM_PREP"), 1e-9);
    assertEquals(0.0, getEventTimestamp(trajectory, "INTAKE"), 1e-9);
  }

  @ParameterizedTest
  @ValueSource(strings = {"SOTM_L_Shoot2ndPass", "SOTM_R_Shoot2ndPass"})
  void secondPassPreparesShooterForThirdPass(String trajectoryName) throws Exception {
    JSONObject trajectory = readTrajectory(trajectoryName);
    assertTrue(getEventTimestamp(trajectory, "SOTM_PREP") > getEventTimestamp(trajectory, "SOTM_END"));
  }

  private static List<String> getEventNames(JSONObject trajectory) {
    JSONArray events = (JSONArray) trajectory.get("events");
    List<String> eventNames = new ArrayList<>();
    for (Object eventObject : events) {
      eventNames.add((String) ((JSONObject) eventObject).get("name"));
    }
    return eventNames;
  }

  private static double getEventTimestamp(JSONObject trajectory, String eventName) {
    JSONArray events = (JSONArray) trajectory.get("events");
    for (Object eventObject : events) {
      JSONObject event = (JSONObject) eventObject;
      if (eventName.equals(event.get("name"))) {
        JSONObject from = (JSONObject) event.get("from");
        return ((Number) from.get("targetTimestamp")).doubleValue();
      }
    }
    throw new IllegalArgumentException("Missing event " + eventName);
  }

  private static JSONObject readTrajectory(String trajectoryName)
      throws IOException, ParseException {
    Path path = Path.of("src", "main", "deploy", "choreo", trajectoryName + ".traj");
    try (var reader = Files.newBufferedReader(path)) {
      return (JSONObject) new JSONParser().parse(reader);
    }
  }
}
