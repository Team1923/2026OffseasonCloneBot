// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//? Testing if I can push to main branch directly...

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerFloorConstants;

public class RollerFloorSubsystem extends SubsystemBase {
  public static enum RollerFloorState {
    REST(RPMToVelocityVoltage(0)),
    SPINNING(RPMToVelocityVoltage(0)),
    EJECT(RPMToVelocityVoltage(0));

    public ControlRequest request;

    private RollerFloorState(ControlRequest request) {
      this.request = request;
    }

    private static VelocityVoltage RPMToVelocityVoltage(double rpm) {
      return new VelocityVoltage(rpm / 60);
    }
  }

  private TalonFX RollerFloor = new TalonFX(RollerFloorConstants.ROLLERFLOOR_ID);

  public RollerFloorSubsystem() {
    RollerFloor.getConfigurator().apply(RollerFloorConstants.ROLLERFLOOR_CONFIG);
  }

  public double getRollerFloorSpeed() {
    return RollerFloor.getVelocity().getValueAsDouble() * 60;
  }

  public void setRollerFloorState(RollerFloorState state) {
    RollerFloor.setControl(state.request);
  }

  public boolean isAtRollerFloorState(RollerFloorState state) {
    if (state.request instanceof VelocityVoltage) {
      VelocityVoltage stateVelo = (VelocityVoltage) state.request;
      return (Math.abs((stateVelo.Velocity * 60) - getRollerFloorSpeed()) < RollerFloorConstants.ROLLERFLOOR_rpmTOLERANCE);
    } else
      return true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
