package frc.robot.lib.sim;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/**
 * Drives a simulated TalonFX from a WPILib {@link DCMotorSim} so the real Phoenix closed-loop
 * control requests (PositionVoltage, VelocityVoltage, VoltageOut, ...) run unchanged in simulation.
 *
 * <p>The motor voltage is read and the rotor position is written in the same sim-state frame, so the
 * motor's configured inversion stays consistent without extra setup.
 */
public final class TalonFXMotorSim {
  public static final double LOOP_PERIOD_SECONDS = 0.02;

  private final TalonFXSimState simState;
  private final DCMotorSim motorSim;
  private final double rotorToMechanismRatio;

  /**
   * @param talon the device to simulate
   * @param gearbox motor model, including the number of motors driving the mechanism
   * @param momentOfInertiaKgMetersSquared mechanism moment of inertia
   * @param rotorToMechanismRatio rotor rotations per mechanism rotation
   */
  public TalonFXMotorSim(
      TalonFX talon,
      DCMotor gearbox,
      double momentOfInertiaKgMetersSquared,
      double rotorToMechanismRatio) {
    this.simState = talon.getSimState();
    this.rotorToMechanismRatio = rotorToMechanismRatio;
    this.motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                gearbox, momentOfInertiaKgMetersSquared, rotorToMechanismRatio),
            gearbox);
  }

  /** Advances the physics by one robot loop and feeds the result back into the TalonFX. */
  public void update() {
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    motorSim.setInputVoltage(simState.getMotorVoltage());
    motorSim.update(LOOP_PERIOD_SECONDS);

    simState.setRawRotorPosition(motorSim.getAngularPositionRotations() * rotorToMechanismRatio);
    simState.setRotorVelocity(
        Units.radiansToRotations(motorSim.getAngularVelocityRadPerSec()) * rotorToMechanismRatio);
  }
}
