package frc.robot.subsystems.rollers;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.RollersConstants;
import frc.robot.lib.sim.TalonFXMotorSim;

/** Simulated rollers: the TalonFX velocity loop runs in Phoenix sim against DC motor physics. */
public class RollersIOSim extends RollersIOTalonFX {
  private final TalonFXMotorSim floorSim =
      new TalonFXMotorSim(
          floorMotor,
          DCMotor.getKrakenX60(1),
          RollersConstants.SIM_MOI_KG_METERS_SQUARED,
          RollersConstants.GEAR_RATIO);

  @Override
  public void updateInputs(RollersIOInputs inputs) {
    floorSim.update();
    super.updateInputs(inputs);
  }
}
