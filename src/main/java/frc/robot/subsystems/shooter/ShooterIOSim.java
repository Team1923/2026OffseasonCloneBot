package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ShooterConstants;
import frc.robot.lib.sim.TalonFXMotorSim;

/**
 * Simulated shooter: the TalonFX control loops run in Phoenix sim against DC motor physics. The
 * drum is modeled as all four motors driving one flywheel, stepped through the leader.
 */
public class ShooterIOSim extends ShooterIOTalonFX {
  private final TalonFXMotorSim hoodSim =
      new TalonFXMotorSim(
          hoodMotor,
          DCMotor.getKrakenX44(1),
          ShooterConstants.HOOD_SIM_MOI_KG_METERS_SQUARED,
          ShooterConstants.HOOD_GEAR_RATIO);
  private final TalonFXMotorSim drumSim =
      new TalonFXMotorSim(
          drumLeader,
          DCMotor.getKrakenX60(DRUM_MOTOR_COUNT),
          ShooterConstants.DRUM_SIM_MOI_KG_METERS_SQUARED,
          ShooterConstants.DRUM_GEAR_RATIO);

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    hoodSim.update();
    drumSim.update();
    for (TalonFX follower : new TalonFX[] {drumFollowerOne, drumFollowerTwo, drumFollowerThree}) {
      follower.getSimState().setSupplyVoltage(RobotController.getBatteryVoltage());
    }
    super.updateInputs(inputs);
  }
}
