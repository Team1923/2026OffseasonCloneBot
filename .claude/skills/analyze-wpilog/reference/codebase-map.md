# CloneBot codebase map (for log analysis and fixes)

The full rules are in `docs/CODEBASE_ARCHITECTURE_PLAYBOOK.md` (read it before a non-trivial fix),
the robot in `docs/ROBOT_OVERVIEW.md`, and logging/replay in `docs/SIMULATION_AND_LOGGING.md`. This
page is the short version, plus the link from each log key to the code that writes it.

If anything here disagrees with the code, the code wins: fix this page. (The analyzer itself reads
mechanisms from the log and limits from `Constants.java`, so it can't drift.)

## How a mechanism is controlled (and where it shows up in the log)

```text
button / auto
  -> subsystem command factory writes StateHandler.desired...State   RealOutputs/StateHandler/.../Desired*
  -> state machine (default command) reads it into a local variable,
     may override it for a safety guard                               RealOutputs/<Mech>/.../AppliedState
  -> subsystem.apply...State(): io.set...Control(state.request)       RealOutputs/<Mech>/.../Setpoint*
  -> TalonFX runs the control request; IO reads sensors               <Mech>/... (inputs, root of the log)
  -> at...State() within tolerance -> state machine writes current    RealOutputs/StateHandler/.../Current*
```

So when something didn't happen, walk the chain in the log:
1. **Desired** never changed: the button/command never ran (check `RealOutputs/Commands/*` and the
   binding in `RobotContainer.java`).
2. Desired changed but **AppliedState** differs: a state machine guard replaced it (on purpose?).
3. Applied but **Setpoint** is 0 or wrong: the state enum's request value (placeholder 0s are common
   right now), or the vision interpolation map is empty.
4. Setpoint right but **measured** doesn't follow: look at AppliedVolts, StatorCurrent, Connected,
   battery. No volts = software (soft limit, request type, disabled). Volts + current but no motion =
   mechanical. Motion the wrong way = inversion config.
5. Measured reaches the target but **Current** state never confirms: tolerance or at-state logic.

## Files

| Path (under `src/main/java/frc/robot/`) | Role |
|---|---|
| `Robot.java` | AdvantageKit start, alliance update before the scheduler, logs `Commands/<name>` |
| `RobotContainer.java` | default commands (state machines) and controller bindings |
| `StateHandler.java` | every desired/current state, scoring mode, swerve state; `log()` writes `StateHandler/*` |
| `Constants.java` | IDs, gear ratios, soft limits, current limits, gains, tolerances, interpolation maps |
| `subsystems/<x>/<X>Subsystem.java` | state enums with their control requests, apply/at-state, command factories |
| `subsystems/<x>/<X>IO.java` | `Inputs` (every logged sensor), `Real` (TalonFX), `Sim` (physics) |
| `statemachines/*StateMachine.java` | default commands: guards, apply, publish current state |
| `commands/RobotCommands.java` | multi-subsystem actions: intake, shoot (ranged/ferry), eject |
| `lib/logging/LoggingStartup.java` | sim/replay selector; env vars `AKIT_LOG_PATH`, `ROBOT_REPLAY_OUTPUT` |
| `lib/swerve/TunerConstants.java` | generated swerve constants (IDs, offsets, gains); regenerate in Tuner X, don't hand-edit casually |

## Controls (RobotContainer)

Driver (Xbox, port 0): left stick drive, right stick X rotate; **right trigger** shoot (ranged or
ferry by scoring mode); **back** eject; **X** reset pose from the best MegaTag1 camera.
Operator (PS5, port 1): **R1** ground intake; **L2** floor roller feed; **circle** ranged mode;
**triangle** ferry mode; **options** stow intake; **square** home the hood.

## Mechanisms

All Phoenix positions/velocities are in **mechanism** units because `SensorToMechanismRatio` is set
to the gear ratio. Requests are rotations (`Units.degreesToRotations`) or rotations/s (`rpm / 60`);
the log shows degrees and RPM. Gains are per mechanism rotation, so velocity kV/kP scale with the
gear ratio.

**Intake arm** (`IntakeSubsystem`, Kraken X60): Motion Magic position. States STOWED / DEPLOYED /
EXTENDED. Zeroed at boot to `ARM_ZERO_OFFSET_DEGREES`, so the arm **must be stowed at power-on**.
kG uses `Arm_Cosine` with `ARM_GRAVITY_OFFSET_DEGREES`: if that offset is wrong the arm sags or
drifts at some angles. That looks like a mechanical problem but is a config one.
Keys: `Intake/Arm*`, `RealOutputs/Intake/Arm/*`.

**Intake rollers** (`IntakeSubsystem`): open-loop `VoltageOut`. States REST / INTAKE / HOLD / EJECT.
`IntakeStateMachine` holds rollers at REST while INTAKE is requested until the arm confirms its
state (that is the "state machine overrode the request" you will see during deploys).
`atRollerState` is always true. Keys: `Intake/Roller*`.

**Hopper floor roller** (`RollersSubsystem`): `VelocityVoltage`. States REST / INTAKE / FEED / EJECT.
`feedWhenAllowedCommand` switches FEED/REST each loop from a condition, which is how shooting gates
feeding. Keys: `Rollers/Floor*`, `RealOutputs/Rollers/Floor/*`.

**Shooter hood** (`ShooterSubsystem`, Kraken X44): Motion Magic position. States STOWED / SHOOTING /
FERRY. **SHOOTING's request object is rewritten every loop** in `ShooterStateMachine` from
`VisionSubsystem.getHoodAngle()` (the `InterpolationConstants.distanceToHoodMap`, 0 while empty).
The TalonFX encoder reads 0 at boot, so the hood is only correct after `homeHoodCommand` (drives
into the 0 degree hard stop; logs `RealOutputs/Shooter/Hood/Homed`). Soft limits apply to every
request, including `VoltageOut`, unless the request sets `IgnoreSoftwareLimits`.

**Shooter drum** (`ShooterSubsystem`, 4x Kraken X60): `VelocityVoltage` on the leader; followers via
`Follower` (one Aligned, two Opposed, set in `ShooterIO.Real`). States IDLE / WARMING / SHOOTING /
FERRY; SHOOTING's RPM is rewritten from `distanceToRPMMap`. Inputs are arrays in order leader,
follower one, two, three. A follower with the wrong alignment fights the others: the drum
accelerates slowly and one motor draws far more current.

**Readiness** (`ShooterSubsystem.readyToShoot`): desired drum state is SHOOTING or FERRY, its
setpoint is above 0, hood within `HOOD_TOLERANCE_DEGREES`, drum within `DRUM_TOLERANCE_RPM`.
Ranged shooting also needs `SwerveSubsystem.isFacing(hub)` (5 degrees) before feeding.

**Swerve** (`SwerveSubsystem`, CTRE generated): `SwerveStateMachine` drives FIELD_CENTRIC /
ROBOT_CENTRIC / FACING_HEADING. Aiming sets FACING_HEADING with a heading toward the alliance hub;
heading control uses blue-alliance perspective (a red-side 180 degree error was fixed before, so
suspect perspective first if aiming is exactly backwards). Inputs are arrays per module.

**Vision** (`VisionSubsystem`): four Limelights' MegaTag2 fused into the pose estimator, rejected
while yawing over 720 deg/s. In replay the pose comes straight from the log, so vision/odometry
changes can't be tested by replay.

## Gotchas that have bitten this code before

- `Command.withTimeout(...)` ends the command **normally**, so `finallyDo(interrupted -> ...)` sees
  `interrupted == false` on timeout. Track success explicitly.
- A soft limit at the encoder's boot value (0) blocks a homing move toward it unless the request
  ignores soft limits.
- State enum requests are shared, mutable objects. Mutating one (as SHOOTING is) changes it for
  every user; never mutate a request in a command.
- Every held command must restore a safe desired state in `finallyDo`/`runEnd`.
- Each `StateHandler` field has one writer: command factories write desired, the state machine
  writes current. Two commands writing the same desired field shows up as rapid state flapping.
- New sensor fields need `toLog` **and** `fromLog`; `LoggableInputsTest` fails otherwise.
- Sim has no gravity, friction, or game pieces: kG, kS, and jams never show up in sim logs.

## Building and replaying

- Build/test: WPILib 2026 JDK (`C:\Users\Public\wpilib\2026\jdk`); the system Java is too new for
  Gradle 8.11 and fails with "Type T not present".
- Headless replay: `scripts/replay.py LOG` (builds the jar + natives, runs without HAL extensions,
  writes `LOG_sim.wpilog`). The PathPlanner "Failed to load ... config" error during replay is
  expected until the PathPlanner robot config exists.
