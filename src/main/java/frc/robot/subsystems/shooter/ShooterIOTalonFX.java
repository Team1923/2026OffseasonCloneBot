package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ShooterConstants;

/** Shooter IO backed by a TalonFX hood and a leader + three follower TalonFX drum. */
public class ShooterIOTalonFX implements ShooterIO {
  protected final TalonFX hoodMotor = new TalonFX(ShooterConstants.HOOD_MOTOR_ID);
  protected final TalonFX drumLeader = new TalonFX(ShooterConstants.DRUM_LEADER_ID);
  protected final TalonFX drumFollowerOne = new TalonFX(ShooterConstants.DRUM_FOLLOWER_ONE_ID);
  protected final TalonFX drumFollowerTwo = new TalonFX(ShooterConstants.DRUM_FOLLOWER_TWO_ID);
  protected final TalonFX drumFollowerThree =
      new TalonFX(ShooterConstants.DRUM_FOLLOWER_THREE_ID);

  /** Same order as the drum array inputs. */
  protected final TalonFX[] drumMotors = {
    drumLeader, drumFollowerOne, drumFollowerTwo, drumFollowerThree
  };

  private final StatusSignal<Angle> hoodPosition = hoodMotor.getPosition();
  private final StatusSignal<AngularVelocity> hoodVelocity = hoodMotor.getVelocity();
  private final StatusSignal<Voltage> hoodAppliedVolts = hoodMotor.getMotorVoltage();
  private final StatusSignal<Current> hoodSupplyCurrent = hoodMotor.getSupplyCurrent();
  private final StatusSignal<Current> hoodStatorCurrent = hoodMotor.getStatorCurrent();
  private final StatusSignal<Temperature> hoodTemperature = hoodMotor.getDeviceTemp();

  private final StatusSignal<AngularVelocity> drumVelocity = drumLeader.getVelocity();
  private final DrumMotorSignals[] drumSignals = new DrumMotorSignals[DRUM_MOTOR_COUNT];

  private final Debouncer hoodConnectedDebounce = new Debouncer(0.5);
  private final Debouncer[] drumConnectedDebounce = new Debouncer[DRUM_MOTOR_COUNT];

  private record DrumMotorSignals(
      StatusSignal<Voltage> appliedVolts,
      StatusSignal<Current> supplyCurrent,
      StatusSignal<Current> statorCurrent,
      StatusSignal<Temperature> temperature) {
    DrumMotorSignals(TalonFX motor) {
      this(
          motor.getMotorVoltage(),
          motor.getSupplyCurrent(),
          motor.getStatorCurrent(),
          motor.getDeviceTemp());
    }

    BaseStatusSignal[] all() {
      return new BaseStatusSignal[] {appliedVolts, supplyCurrent, statorCurrent, temperature};
    }
  }

  public ShooterIOTalonFX() {
    hoodMotor.getConfigurator().apply(ShooterConstants.HOOD_CONFIG);
    for (TalonFX drumMotor : drumMotors) {
      drumMotor.getConfigurator().apply(ShooterConstants.DRUM_CONFIG);
    }

    // TODO: Verify follower directions on the assembled drum before enabling it.
    drumFollowerOne.setControl(
        new Follower(drumLeader.getDeviceID(), MotorAlignmentValue.Aligned));
    drumFollowerTwo.setControl(
        new Follower(drumLeader.getDeviceID(), MotorAlignmentValue.Opposed));
    drumFollowerThree.setControl(
        new Follower(drumLeader.getDeviceID(), MotorAlignmentValue.Opposed));

    for (int i = 0; i < DRUM_MOTOR_COUNT; i++) {
      drumSignals[i] = new DrumMotorSignals(drumMotors[i]);
      drumConnectedDebounce[i] = new Debouncer(0.5);
    }
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var hoodStatus =
        BaseStatusSignal.refreshAll(
            hoodPosition,
            hoodVelocity,
            hoodAppliedVolts,
            hoodSupplyCurrent,
            hoodStatorCurrent,
            hoodTemperature);

    inputs.hoodConnected = hoodConnectedDebounce.calculate(hoodStatus.isOK());
    inputs.hoodPositionDegrees = Units.rotationsToDegrees(hoodPosition.getValueAsDouble());
    inputs.hoodVelocityDegreesPerSecond =
        Units.rotationsToDegrees(hoodVelocity.getValueAsDouble());
    inputs.hoodAppliedVolts = hoodAppliedVolts.getValueAsDouble();
    inputs.hoodSupplyCurrentAmps = hoodSupplyCurrent.getValueAsDouble();
    inputs.hoodStatorCurrentAmps = hoodStatorCurrent.getValueAsDouble();
    inputs.hoodTemperatureCelsius = hoodTemperature.getValueAsDouble();

    BaseStatusSignal.refreshAll(drumVelocity);
    inputs.drumVelocityRpm = drumVelocity.getValueAsDouble() * 60.0;

    for (int i = 0; i < DRUM_MOTOR_COUNT; i++) {
      DrumMotorSignals signals = drumSignals[i];
      var status = BaseStatusSignal.refreshAll(signals.all());
      inputs.drumConnected[i] = drumConnectedDebounce[i].calculate(status.isOK());
      inputs.drumAppliedVolts[i] = signals.appliedVolts().getValueAsDouble();
      inputs.drumSupplyCurrentAmps[i] = signals.supplyCurrent().getValueAsDouble();
      inputs.drumStatorCurrentAmps[i] = signals.statorCurrent().getValueAsDouble();
      inputs.drumTemperatureCelsius[i] = signals.temperature().getValueAsDouble();
    }
  }

  @Override
  public void setHoodControl(ControlRequest request) {
    hoodMotor.setControl(request);
  }

  @Override
  public void setDrumControl(ControlRequest request) {
    drumLeader.setControl(request);
  }
}
