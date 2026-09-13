// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.LoggingConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.statemachines.IntakeStateMachine;
import frc.robot.statemachines.RollersStateMachine;
import frc.robot.statemachines.ShooterStateMachine;
import frc.robot.statemachines.SwerveStateMachine;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersSubsystem;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private StateHandler stateHandler = StateHandler.getInstance();

  private CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private CommandXboxController operatorController =
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  // Swerve picks its own IO for the current mode inside getInstance().
  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private final VisionSubsystem visionSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private final RollersSubsystem rollersSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (LoggingConstants.CURRENT_MODE) {
      case REAL -> {
        visionSubsystem =
            new VisionSubsystem(
                new VisionIO.Limelight(VisionConstants.FRONT_LEFT_LIMELIGHT),
                new VisionIO.Limelight(VisionConstants.FRONT_RIGHT_LIMELIGHT),
                new VisionIO.Limelight(VisionConstants.REAR_LEFT_LIMELIGHT),
                new VisionIO.Limelight(VisionConstants.REAR_RIGHT_LIMELIGHT));
        intakeSubsystem = new IntakeSubsystem(new IntakeIO.Real());
        rollersSubsystem = new RollersSubsystem(new RollersIO.Real());
        shooterSubsystem = new ShooterSubsystem(new ShooterIO.Real());
      }
      case SIM -> {
        // Limelights are not simulated; vision contributes no measurements in sim.
        visionSubsystem =
            new VisionSubsystem(new VisionIO() {}, new VisionIO() {}, new VisionIO() {}, new VisionIO() {});
        intakeSubsystem = new IntakeSubsystem(new IntakeIO.Sim());
        rollersSubsystem = new RollersSubsystem(new RollersIO.Sim());
        shooterSubsystem = new ShooterSubsystem(new ShooterIO.Sim());
      }
      default -> {
        // Replay: no hardware, every input comes from the log.
        visionSubsystem =
            new VisionSubsystem(new VisionIO() {}, new VisionIO() {}, new VisionIO() {}, new VisionIO() {});
        intakeSubsystem = new IntakeSubsystem(new IntakeIO() {});
        rollersSubsystem = new RollersSubsystem(new RollersIO() {});
        shooterSubsystem = new ShooterSubsystem(new ShooterIO() {});
      }
    }

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Default Commands
    swerveSubsystem.setDefaultCommand(
        new SwerveStateMachine(
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            swerveSubsystem));
    intakeSubsystem.setDefaultCommand(new IntakeStateMachine(intakeSubsystem));
    rollersSubsystem.setDefaultCommand(new RollersStateMachine(rollersSubsystem));
    shooterSubsystem.setDefaultCommand(new ShooterStateMachine(shooterSubsystem));

    // Driver Bindings
    // Left unassigned until the driver control map is specified. Bind public subsystem
    // command factories or compositions from RobotCommands here.

    // Operator Bindings
    // Left unassigned until the operator control map is specified. Bind public subsystem
    // command factories or compositions from RobotCommands here.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
