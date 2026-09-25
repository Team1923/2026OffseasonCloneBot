// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statemachines;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.StateHandler;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.function.DoubleSupplier;

/** Default teleop drive command with field-, robot-, and heading-centric modes. */
public final class SwerveStateMachine extends Command {
  private static final SwerveRequest.FieldCentric FIELD_CENTRIC_REQUEST =
      new SwerveRequest.FieldCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.Position);

  private static final SwerveRequest.RobotCentric ROBOT_CENTRIC_REQUEST =
      new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.Velocity)
          .withSteerRequestType(SteerRequestType.Position);

  private static final SwerveRequest.FieldCentricFacingAngle FACING_HEADING_REQUEST =
      createFacingHeadingRequest();

  private final DoubleSupplier translationSupplier;
  private final DoubleSupplier strafeSupplier;
  private final DoubleSupplier rotationSupplier;
  private final SwerveSubsystem swerve;
  private final StateHandler stateHandler = StateHandler.getInstance();

  public SwerveStateMachine(
      DoubleSupplier translationSupplier,
      DoubleSupplier strafeSupplier,
      DoubleSupplier rotationSupplier,
      SwerveSubsystem swerve) {
    this.translationSupplier = translationSupplier;
    this.strafeSupplier = strafeSupplier;
    this.rotationSupplier = rotationSupplier;
    this.swerve = swerve;

    addRequirements(swerve);
    setName("SwerveStateMachine");
  }

  @Override
  public void execute() {
    double translation = deadband(translationSupplier.getAsDouble());
    double strafe = deadband(strafeSupplier.getAsDouble());
    double rotation = deadband(rotationSupplier.getAsDouble());

    // Path-following commands require swerve and normally interrupt this default command. This
    // fallback prevents driver input if the default command is ever scheduled during autonomous.
    if (DriverStation.isAutonomous()) {
      translation = 0;
      strafe = 0;
      rotation = 0;
    }

    double velocityX = translation * stateHandler.maxDriveSpeedMetersPerSecond;
    double velocityY = strafe * stateHandler.maxDriveSpeedMetersPerSecond;
    double rotationalRate = rotation * SwerveConstants.MAX_DRIVE_ANGULAR_VELOCITY;

    SwerveRequest request =
        switch (stateHandler.swerveState) {
          case FIELD_CENTRIC ->
              FIELD_CENTRIC_REQUEST
                  .withVelocityX(velocityX)
                  .withVelocityY(velocityY)
                  .withRotationalRate(rotationalRate);
          case ROBOT_CENTRIC ->
              ROBOT_CENTRIC_REQUEST
                  .withVelocityX(velocityX)
                  .withVelocityY(velocityY)
                  .withRotationalRate(rotationalRate);
          case FACING_HEADING ->
              FACING_HEADING_REQUEST
                  .withVelocityX(velocityX)
                  .withVelocityY(velocityY)
                  .withTargetDirection(stateHandler.desiredSwerveHeading);
        };

    swerve.setControl(request);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private static double deadband(double input) {
    return MathUtil.applyDeadband(input, ControllerConstants.DRIVER_DEADBAND);
  }

  private static SwerveRequest.FieldCentricFacingAngle createFacingHeadingRequest() {
    var request =
        new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.Position)
            // desiredSwerveHeading is a blue-origin field angle. Without this, CTRE also rotates it
            // by the operator perspective, so on red the robot faces 180 degrees away from the
            // target. Translation still follows the operator perspective.
            .withTargetDirectionPerspective(
                SwerveRequest.TargetDirectionPerspectiveValue.BlueAlliance);
    request.HeadingController = SwerveConstants.HEADING_CONTROLLER;
    return request;
  }
}
