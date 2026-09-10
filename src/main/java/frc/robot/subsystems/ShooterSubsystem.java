// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  public static enum ShooterHoodState {
    STOWED(PositionVoltDegrees(0)), // 0 is placeholder, would be the stowed position of the hood
    TRACKING(PositionVoltDegrees(0)), // 0 is placeholder, filled in from table
    FERRY(PositionVoltDegrees(0));  // 0 is placeholder, would be the position of the hood for ferrying FUEL

    public final PositionVoltage request;

    private ShooterHoodState(PositionVoltage request) {
      this.request = request;
    }

    private static PositionVoltage PositionVoltDegrees(double degrees) {
      return new PositionVoltage(Units.degreesToRotations(degrees)); // Convert degrees to rotations for PositionVoltage
    }
  }

  public static enum ShooterDrumState {
    IDLE(VeloVoltRPM(0)),
    WARMING(VeloVoltRPM(0)), // 0 is placeholder, would be slow velocity of the wheels (warming up)
    SHOOTING(VeloVoltRPM(0)), // 0 is placeholder, would be the velocity of the wheels for shooting
    FERRY(VeloVoltRPM(0)); // 0 is placeholder, would be the velocity of the wheels for ferrying FUEL

    public final VelocityVoltage request;

    private ShooterDrumState(VelocityVoltage request) {
      this.request = request;
    }
    
    private static VelocityVoltage VeloVoltRPM(double rpm) {
      return new VelocityVoltage(rpm / 60.0); // Convert RPM to RPS for VelocityVoltage
    }
  }

  private final TalonFX shooterHoodMotor = new TalonFX(0); // Placeholder ID for the hood motor
  private final TalonFX shooterDrumMotor = new TalonFX(1); // Placeholder ID for the drum motor
  private final TalonFX shooterDrumFollowerMotor = new TalonFX(2); // follower
  private final TalonFX shooterDrumFollowerMotor2 = new TalonFX(3); // follower
  private final TalonFX shooterDrumFollowerMotor3 = new TalonFX(4); // follower

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
