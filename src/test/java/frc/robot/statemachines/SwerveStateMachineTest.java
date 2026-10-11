package frc.robot.statemachines;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.TargetDirectionPerspectiveValue;
import org.junit.jupiter.api.Test;

class SwerveStateMachineTest {
  /**
   * Desired headings are computed from blue-origin poses. If heading control used the operator
   * perspective, CTRE would rotate the target by the operator forward direction, which is 180
   * degrees on the red alliance, and the robot would aim exactly away from the target.
   */
  @Test
  void headingTargetIgnoresOperatorPerspective() {
    SwerveRequest.FieldCentricFacingAngle request =
        SwerveStateMachine.createFacingHeadingRequest();

    assertEquals(
        TargetDirectionPerspectiveValue.BlueAlliance,
        request.TargetDirectionPerspective,
        "heading target must stay in blue-origin coordinates");
  }

  /**
   * The heading controller must wrap at -180..180 degrees or the heading error can take the long
   * way around. CTRE's own controller does this, which is why gains are set with withHeadingPID
   * instead of assigning a replacement controller.
   */
  @Test
  void headingControllerWrapsAtHalfTurn() {
    SwerveRequest.FieldCentricFacingAngle request =
        SwerveStateMachine.createFacingHeadingRequest();

    assertTrue(
        request.HeadingController.isContinuousInputEnabled(),
        "heading controller must wrap so it always turns the short way");
  }
}
