// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

  private static IntakeSubsystem intakeSubsystem;

  public static enum IntakeArmState {
    STOWED(degreesToPositionVoltage(-128)),
    DEPLOYED(degreesToPositionVoltage(0)),
    EXTENDED(degreesToPositionVoltage(-30)),
    FUEL_PUSH_HOLD(degreesToPositionVoltage(-40)),
    FUEL_PUSH_MOVE(new DutyCycleOut(-0.2));

    public ControlRequest request;

    private IntakeArmState(ControlRequest request) {
      this.request = request;
    }

    private static PositionVoltage degreesToPositionVoltage(double degrees) {
      return new PositionVoltage(Units.degreesToRotations(degrees));
    }
  }

  

  public static enum IntakeRollerStates {
    EJECT(new VoltageOut(0)),
    INTAKE(new VoltageOut(8.5)),
    PUSH(new VoltageOut(4)),
    REST(new VoltageOut(0)),
    EXTENDED(new VoltageOut(1));

    public VoltageOut request;

    private IntakeRollerStates(VoltageOut request) {
      this.request = request;
    }

  }

  public static synchronized IntakeSubsystem getInstance() {
    if (intakeSubsystem == null) {
      intakeSubsystem = new IntakeSubsystem();
    }
    return intakeSubsystem;
  }

  private TalonFX intakeArm = new TalonFX(Constants.IntakeConstants.intakeArmID);
  private TalonFX intakeRollers = new TalonFX(Constants.IntakeConstants.intakeRollerID);

  /** Creates a new IntakeSubsystem. */
  private IntakeSubsystem() {
    intakeArm.getConfigurator().apply(Constants.IntakeConstants.INTAKE_ARM_CONFIGS);

    intakeRollers.getConfigurator().apply(Constants.IntakeConstants.INTAKE_ROLLER_CONFIGS);

    zeroIntakeArm();
  }

  public double getArmPosition() {
    return Units.rotationsToDegrees(intakeArm.getPosition().getValueAsDouble());
  }

  public double getRollerSpeed() {
    return (intakeRollers.getVelocity().getValueAsDouble()) / 60;
  }

  public void setIntakeArmState(IntakeArmState state) {
    intakeArm.setControl(state.request);
  }

  public void setIntakeRollerStates(IntakeRollerStates state) {
    intakeRollers.setControl(state.request);
  }

  public boolean isAtIntakeArmState(IntakeArmState state) {
    if (state.request instanceof PositionVoltage) {
      PositionVoltage positionReq = (PositionVoltage) state.request;
      return Math.abs(getArmPosition()
          - Units.rotationsToDegrees(positionReq.Position)) < Constants.IntakeConstants.intakeArmAllowance;

    } else {
      return true;
    }
  }

  public boolean isAtIntakeRollerState(IntakeRollerStates state) {
    return true;
  }

  public void setIntakeArmControl(ControlRequest request) {
    intakeArm.setControl(request);
  }

  public void zeroIntakeArm() {
    intakeArm.setPosition(Units.degreesToRotations(Constants.IntakeConstants.intakeArmZeroOffset));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}