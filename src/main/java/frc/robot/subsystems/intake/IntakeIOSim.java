package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Constants.IntakeConstants;
import frc.robot.lib.sim.TalonFXMotorSim;

/** Simulated intake: the TalonFX control loops run in Phoenix sim against DC motor physics. */
public class IntakeIOSim extends IntakeIOTalonFX {
  private final TalonFXMotorSim armSim =
      new TalonFXMotorSim(
          arm,
          DCMotor.getKrakenX60(1),
          IntakeConstants.ARM_SIM_MOI_KG_METERS_SQUARED,
          IntakeConstants.ARM_GEAR_RATIO);
  private final TalonFXMotorSim rollerSim =
      new TalonFXMotorSim(
          roller,
          DCMotor.getKrakenX60(1),
          IntakeConstants.ROLLER_SIM_MOI_KG_METERS_SQUARED,
          IntakeConstants.ROLLER_GEAR_RATIO);

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    armSim.update();
    rollerSim.update();
    super.updateInputs(inputs);
  }
}
