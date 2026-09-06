// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.statemachines;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.StateHandler;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.SwerveStates;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SwerveStateMachine extends Command {

  public static class Requests {

    /* Swerve requests to apply during SysId characterization */
    public static final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    public static final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    public static final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();

    private static final SwerveRequest.FieldCentric FIELD_CENTRIC = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.Position);

    private static final SwerveRequest.RobotCentric ROBOT_CENTRIC = new SwerveRequest.RobotCentric();

  }

  SwerveSubsystem swerveSubsystem;

  StateHandler stateHandler = StateHandler.getStateHandler();

  private DoubleSupplier translationSupplier;
  private DoubleSupplier strafeSupplier;
  private DoubleSupplier rotationSupplier;

  /** Creates a new SwerveStateMachine. */
  public SwerveStateMachine(DoubleSupplier translationSupplier, DoubleSupplier strafeSupplier,
      DoubleSupplier rotationSupplier, SwerveSubsystem swerveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.translationSupplier = translationSupplier;
    this.strafeSupplier = strafeSupplier;
    this.rotationSupplier = rotationSupplier;

    this.swerveSubsystem = swerveSubsystem;

    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveStates currentState = stateHandler.swerveState;
    SwerveRequest desiredRequest = Requests.FIELD_CENTRIC;

    double translation = MathUtil.applyDeadband(translationSupplier.getAsDouble(),
        Constants.ControllerConstants.driverDeadband);

    double strafe = MathUtil.applyDeadband(strafeSupplier.getAsDouble(),
        Constants.ControllerConstants.driverDeadband);

    double rotation = MathUtil.applyDeadband(rotationSupplier.getAsDouble(),
        Constants.ControllerConstants.driverDeadband);

    switch (currentState) {
      case FIELD_CENTRIC:
        if (DriverStation.isAutonomous()) {
          desiredRequest = Requests.FIELD_CENTRIC
              .withVelocityX(0)
              .withVelocityY(0)
              .withRotationalRate(0);
        } else {
          desiredRequest = Requests.FIELD_CENTRIC
              .withVelocityX(translation * stateHandler.maxDriveSpeedMetersPerSecond)
              .withVelocityY(strafe * stateHandler.maxDriveSpeedMetersPerSecond)
              .withRotationalRate(rotation * SwerveConstants.MAX_DRIVE_ANGULAR_VELOCITY);

        }
        break;
      default:
        break;
    }
    swerveSubsystem.setControl(desiredRequest);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static SwerveStateMachine generateHoldStillSwerveStateMachine() {
    return new SwerveStateMachine(() -> 0, () -> 0, () -> 0, SwerveSubsystem.getInstance());
  }
}
