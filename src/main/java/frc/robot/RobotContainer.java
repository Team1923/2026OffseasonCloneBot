// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.RobotCommands;
import frc.robot.statemachines.IntakeStateMachine;
import frc.robot.statemachines.RollersStateMachine;
import frc.robot.statemachines.ShooterStateMachine;
import frc.robot.statemachines.SwerveStateMachine;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RollersSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

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

  private SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private VisionSubsystem visionSubsystem = VisionSubsystem.getInstance();
  private IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private RollersSubsystem rollersSubsystem = new RollersSubsystem();
  private ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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
    rollersSubsystem.setDefaultCommand(
      rollersSubsystem.feedWhenAllowedCommand(shooterSubsystem::readyToShoot));


    // Driver Bindings
    // command factories or compositions from RobotCommands here.
    driverController.leftTrigger().whileTrue(intakeSubsystem.intakeCommand());
    driverController.rightTrigger().whileTrue(RobotCommands.shootCommand(shooterSubsystem, swerveSubsystem, rollersSubsystem, null));
    driverController.leftBumper().whileTrue(shooterSubsystem.ferryCommand());
    driverController.back().whileTrue(RobotCommands.ejectCommand(intakeSubsystem, rollersSubsystem));

    // Operator Bindings
    // Left unassigned until the operator control map is specified. Bind public subsystem
    // command factories or compositions from RobotCommands here.
    
    /* TODO: Ishaan complete operator bindings */
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
