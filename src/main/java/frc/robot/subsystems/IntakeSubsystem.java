// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.StateHandler;

/** Controls the intake pivot and rollers. Hardware is applied by {@code IntakeStateMachine}. */
public final class IntakeSubsystem extends SubsystemBase {
  public enum IntakeArmState {
    STOWED(positionRequest(-128)),
    DEPLOYED(positionRequest(0)),
    EXTENDED(positionRequest(-30));

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
    INTAKE(new VoltageOut(8.5)),
    HOLD(new VoltageOut(1.0)),
    EJECT(new VoltageOut(-8.5));

    public final VoltageOut request;

    IntakeRollerState(VoltageOut request) {
      this.request = request;
    }
  }

  private final StateHandler stateHandler = StateHandler.getInstance();
  private final TalonFX intakeArm = new TalonFX(IntakeConstants.ARM_MOTOR_ID);
  private final TalonFX intakeRollers = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID);

  public IntakeSubsystem() {
    intakeArm.getConfigurator().apply(IntakeConstants.ARM_CONFIG);
    intakeRollers.getConfigurator().apply(IntakeConstants.ROLLER_CONFIG);
    zeroIntakeArm();
  }

  public double getArmPositionDegrees() {
    return Units.rotationsToDegrees(intakeArm.getPosition().getValueAsDouble());
  }

  public double getRollerSpeedRpm() {
    return intakeRollers.getVelocity().getValueAsDouble() * 60.0;
  }

  /** Applies the desired arm state. Ordinary commands request states instead of calling this. */
  public void applyArmState(IntakeArmState desiredState) {
    intakeArm.setControl(desiredState.request);
  }

  /** Applies the desired roller state. Ordinary commands request states instead of calling this. */
  public void applyRollerState(IntakeRollerState desiredState) {
    intakeRollers.setControl(desiredState.request);
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
    intakeArm.setPosition(Units.degreesToRotations(IntakeConstants.ARM_ZERO_OFFSET_DEGREES));
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
