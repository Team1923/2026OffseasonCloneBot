// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.StateHandler;
import org.littletonrobotics.junction.Logger;

/** Controls the intake pivot and rollers. Hardware is applied by {@code IntakeStateMachine}. */
public final class IntakeSubsystem extends SubsystemBase {
  private static IntakeSubsystem intakeSubsystem;

  public static synchronized IntakeSubsystem getInstance() {
    if (intakeSubsystem == null) {
      intakeSubsystem =
          new IntakeSubsystem(
              switch (StateHandler.getInstance().robotMode) {
                case REAL -> new IntakeIO.Real();
                case SIM -> new IntakeIO.Sim();
                case REPLAY -> new IntakeIO() {};
              });
    }
    return intakeSubsystem;
  }

  public enum IntakeArmState {
    STOWED(positionRequest(0)),
    DEPLOYED(positionRequest(0)),
    EXTENDED(positionRequest(0));

    public final PositionVoltage request;

    IntakeArmState(PositionVoltage request) {
      this.request = request;
    }

    private static PositionVoltage positionRequest(double degrees) {
      return new PositionVoltage(Units.degreesToRotations(degrees));
    }
  }

  public enum IntakeRollerState {
    REST(new VoltageOut(0)),
    INTAKE(new VoltageOut(0)),
    HOLD(new VoltageOut(0)),
    EJECT(new VoltageOut(0));

    public final VoltageOut request;

    IntakeRollerState(VoltageOut request) {
      this.request = request;
    }
  }

  private final StateHandler stateHandler = StateHandler.getInstance();
  private final IntakeIO io;
  private final IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();

  private final VoltageOut armHomingRequest =
      new VoltageOut(IntakeConstants.ARM_HOMING_VOLTS).withIgnoreSoftwareLimits(true);
  private final NeutralOut armNeutralRequest = new NeutralOut();
  private boolean armHomed = false;

  private final Alert armDisconnectedAlert =
      new Alert("Intake arm motor disconnected", AlertType.kError);
  private final Alert rollerDisconnectedAlert =
      new Alert("Intake roller motor disconnected", AlertType.kError);

  private IntakeSubsystem(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    armDisconnectedAlert.set(!inputs.armConnected);
    rollerDisconnectedAlert.set(!inputs.rollerConnected);
    Logger.recordOutput("Intake/Arm/Homed", armHomed);
  }

  public double getArmPositionDegrees() {
    return inputs.armPositionDegrees;
  }

  public double getRollerSpeedRpm() {
    return inputs.rollerVelocityRpm;
  }

  /** Applies the desired arm state. Ordinary commands request states instead of calling this. */
  public void applyArmState(IntakeArmState desiredState) {
    io.setArmControl(desiredState.request);
    Logger.recordOutput("Intake/Arm/AppliedState", desiredState);
    Logger.recordOutput(
        "Intake/Arm/SetpointDegrees", Units.rotationsToDegrees(desiredState.request.Position));
  }

  /** Applies the desired roller state. Ordinary commands request states instead of calling this. */
  public void applyRollerState(IntakeRollerState desiredState) {
    io.setRollerControl(desiredState.request);
    Logger.recordOutput("Intake/Roller/AppliedState", desiredState);
    Logger.recordOutput("Intake/Roller/SetpointVolts", desiredState.request.Output);
  }

  public boolean atArmState(IntakeArmState desiredState) {
    double targetDegrees = Units.rotationsToDegrees(desiredState.request.Position);
    return Math.abs(getArmPositionDegrees() - targetDegrees)
        < IntakeConstants.ARM_POSITION_TOLERANCE_DEGREES;
  }

  public boolean atRollerState(IntakeRollerState desiredState) {
    return true; // will get to state instantly
  }

  public void zeroIntakeArm() {
    io.setArmPosition(IntakeConstants.ARM_ZERO_OFFSET_DEGREES);
  }

  /** Whether the arm has been zeroed against its hard stop since robot code started. */
  public boolean isArmHomed() {
    return armHomed;
  }

  /** Leaves the arm unpowered. The state machine uses this until the arm has been homed. */
  public void stopArm() {
    io.setArmControl(armNeutralRequest);
  }

  /**
   * Drives the arm into its stowed hard stop and zeroes the encoder there. The TalonFX keeps its
   * position across robot-code restarts, so zeroing at startup would make wherever the arm happens
   * to be the new zero. Requires the intake, so the state machine pauses while this runs.
   */
  public Command homeArmCommand() {
    Debouncer stallDebouncer =
        new Debouncer(IntakeConstants.ARM_HOMING_STALL_SECONDS, DebounceType.kRising);
    return startRun(
            () -> stallDebouncer.calculate(false), () -> io.setArmControl(armHomingRequest))
        .until(() -> stallDebouncer.calculate(isArmStalled()))
        .finallyDo(
            interrupted -> {
              io.setArmControl(armNeutralRequest);
              if (interrupted) {
                DriverStation.reportWarning(
                    "Intake arm homing did not finish; the arm stays unpowered until it homes",
                    false);
              } else {
                zeroIntakeArm();
                armHomed = true;
              }
            })
        .withTimeout(IntakeConstants.ARM_HOMING_TIMEOUT_SECONDS)
        .withName("IntakeHomeArm");
  }

  private boolean isArmStalled() {
    return Math.abs(inputs.armStatorCurrentAmps) > IntakeConstants.ARM_HOMING_STALL_CURRENT_AMPS
        && Math.abs(inputs.armVelocityDegreesPerSecond)
            < IntakeConstants.ARM_HOMING_STALL_VELOCITY_DEGREES_PER_SECOND;
  }

  public Command intakeCommand() {
    return Commands.run(
            () -> request(IntakeArmState.DEPLOYED, IntakeRollerState.INTAKE))
        .finallyDo(
            interrupted -> request(IntakeArmState.EXTENDED, IntakeRollerState.HOLD))
        .withName("IntakeCollect");
  }

  public Command stowCommand() {
    return Commands.run(
            () -> request(IntakeArmState.STOWED, IntakeRollerState.REST))
        .until(
            () ->
                stateHandler.currentIntakeArmState == IntakeArmState.STOWED
                    && stateHandler.currentIntakeRollerState == IntakeRollerState.REST)
        .withName("IntakeStow");
  }

  public Command ejectCommand() {
    return Commands.run(
            () -> request(IntakeArmState.DEPLOYED, IntakeRollerState.EJECT))
        .finallyDo(
            interrupted -> request(IntakeArmState.EXTENDED, IntakeRollerState.HOLD))
        .withName("IntakeEject");
  }

  public Command restCommand() {
    return Commands.runOnce(
            () -> request(IntakeArmState.STOWED, IntakeRollerState.REST))
        .withName("IntakeRest");
  }

  private void request(IntakeArmState armState, IntakeRollerState rollerState) {
    stateHandler.desiredIntakeArmState = armState;
    stateHandler.desiredIntakeRollerState = rollerState;
  }
}
