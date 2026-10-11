// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.StateHandler.ScoringMode;
import frc.robot.commands.RobotCommands;
import frc.robot.statemachines.IntakeStateMachine;
import frc.robot.statemachines.RollersStateMachine;
import frc.robot.statemachines.ShooterStateMachine;
import frc.robot.statemachines.SwerveStateMachine;
import frc.robot.subsystems.InfoSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.rollers.RollersSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private CommandXboxController driverController = new CommandXboxController(0);
  private CommandPS5Controller operatorController = new CommandPS5Controller(1);

  private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
  private final VisionSubsystem visionSubsystem = VisionSubsystem.getInstance();
  private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
  private final RollersSubsystem rollersSubsystem = RollersSubsystem.getInstance();
  private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();
  private final InfoSubsystem infoSubsystem = new InfoSubsystem();
  private final StateHandler stateHandler = StateHandler.getInstance();

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
    rollersSubsystem.setDefaultCommand(new RollersStateMachine(rollersSubsystem));
    shooterSubsystem.setDefaultCommand(
        new ShooterStateMachine(shooterSubsystem, visionSubsystem));

    // Home the intake arm on the first enable, and again on the next enable if homing failed. The
    // operator can also retry it without a disable (create button).
    new Trigger(DriverStation::isEnabled)
        .and(() -> !intakeSubsystem.isArmHomed())
        .onTrue(intakeSubsystem.homeArmCommand());

    // Driver Bindings
    driverController
        .x()
        .onTrue(
            swerveSubsystem
                .runOnce(() -> visionSubsystem.zeroPoseToMT1())
                .ignoringDisable(true));
    Trigger shootTrigger = driverController.rightTrigger();
    shootTrigger
        .and(() -> stateHandler.scoringMode == ScoringMode.RANGED)
        .whileTrue(RobotCommands.shootCommand(ScoringMode.RANGED));
    shootTrigger
        .and(() -> stateHandler.scoringMode == ScoringMode.FERRY)
        .whileTrue(RobotCommands.shootCommand(ScoringMode.FERRY));
    driverController.back().whileTrue(RobotCommands.ejectCommand(intakeSubsystem, rollersSubsystem));

    // Operator Bindings
    operatorController.R1().whileTrue(RobotCommands.groundIntake(intakeSubsystem, rollersSubsystem));
    operatorController.L2().whileTrue(rollersSubsystem.feedCommand());
    operatorController.circle().onTrue(stateHandler.setScoringMode(ScoringMode.RANGED));
    operatorController.triangle().onTrue(stateHandler.setScoringMode(ScoringMode.FERRY));
    operatorController.options().onTrue(intakeSubsystem.stowCommand());
    operatorController.square().onTrue(shooterSubsystem.homeHoodCommand());
    operatorController.create().onTrue(intakeSubsystem.homeArmCommand());
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
