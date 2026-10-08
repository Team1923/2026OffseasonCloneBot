package frc.robot.lib.auto;

import static org.junit.jupiter.api.Assertions.assertNotNull;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.wpilibj2.command.Commands;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

class ChoreoTrajectoryTest {
  @BeforeAll
  static void registerNamedCommands() {
    NamedCommands.registerCommand("SOTM", Commands.none());
    NamedCommands.registerCommand("SOTM_END", Commands.none());
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
}
