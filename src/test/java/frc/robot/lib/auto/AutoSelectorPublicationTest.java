package frc.robot.lib.auto;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.junit.jupiter.api.Test;

class AutoSelectorPublicationTest {
  @Test
  void publishesChooserOptionsAtDashboardPath() {
    new AutoInstantiateSelector();
    SmartDashboard.updateValues();

    NetworkTable chooserTable =
        NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getSubTable("Auto Mode");

    assertArrayEquals(
        new String[] {"SOTM LDoubleNZ", "SOTM RDoubleNZ"},
        chooserTable.getEntry("options").getStringArray(new String[0]));
  }
}

//hello world
