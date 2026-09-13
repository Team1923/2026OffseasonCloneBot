# 2026 Rebuilt 9470 Offseason code architecture

This is the implementation contract for coding agents working on the robot.
Read [`ROBOT_OVERVIEW.md`](ROBOT_OVERVIEW.md) for the physical robot and
[`IMPLEMENTATION_GUIDE.md`](IMPLEMENTATION_GUIDE.md) for the current build plan
and examples.

## Project structure

```text
src/main/java/frc/robot/
├── Robot.java                 lifecycle, scheduler, and AdvantageKit setup only
├── RobotContainer.java        construct once (choosing IO per mode), defaults, and bindings
├── Constants.java             one nested constants class per subsystem (+ LoggingConstants)
├── StateHandler.java          desired/current states and shared robot facts
├── subsystems/<name>/         one package per subsystem:
│   ├── <Name>Subsystem.java     states, status methods, logging, simple commands
│   └── <Name>IO.java            inputs, IO interface, and its Real and Sim classes
├── statemachines/             continuous mechanism arbitration
├── commands/                  multi-subsystem or complicated actions
└── lib/                       generated and reusable helpers
```

Do not add mechanism logic to `Robot`, `RobotContainer`, or controller
bindings. Those files wire behavior together; they do not implement it.

## Control model

State-controlled mechanisms use this loop:

```text
button or auto
  -> subsystem command factory (requests a state)
  -> StateHandler desired state
  -> default state machine (applies local guards)
  -> subsystem hardware method
  -> at-state check
  -> StateHandler current state
```

Keep these meanings separate:

- **Desired state:** what the operator, auto, or composed command asks for, stored
  in `StateHandler`. Each cycle the state machine reads it into a local
  `desired…State` variable, which it may adjust for a local safety interlock.
- **Current state:** the desired state confirmed at the hardware by sensors and
  tolerances, written back to `StateHandler`.

A state machine must not overwrite the `StateHandler` desired field to enforce an
interlock. Adjust its local `desired…State` variable instead (this is why the
local keeps the `desired` name even after the machine changes it). This preserves
the original request and prevents multiple commands from fighting over the same
field.

## Where commands belong

Put a command factory in its subsystem when it controls only that subsystem:

```java
public Command intakeCommand() {
  return Commands.run(() -> state.desiredIntakeRollerState = INTAKE)
      .finallyDo(interrupted -> state.desiredIntakeRollerState = REST);
}
```

Put a command in `commands/` when it:

- coordinates two or more subsystems;
- has a substantial sequence, deadline, retry, or conditional flow; or
- represents a complete robot action such as shooting while aiming.

Command rules:

- Return a fresh command from every factory call.
- Every held command must restore a safe desired state when interrupted.
- Bind public commands in `RobotContainer`; do not write desired states there.
- Name commands so logs identify the action.

An intent command that only writes a desired state does **not** require the
physical subsystem; its default state machine must remain scheduled to command
the hardware. A direct manual, zeroing, SysId, or recovery command does require
the subsystem, interrupts the state machine, controls hardware directly, and
leaves both hardware and shared state coherent when it ends.

Commands v2 references:

- [Organizing command-based projects](https://docs.wpilib.org/en/latest/docs/software/commandbased/commands-v2/organizing-command-based.html)
- [Commands and lifecycle](https://docs.wpilib.org/en/latest/docs/software/commandbased/commands-v2/commands.html)
- [Command compositions](https://docs.wpilib.org/en/latest/docs/software/commandbased/commands-v2/command-compositions.html)
- [Subsystems and default commands](https://docs.wpilib.org/en/latest/docs/software/commandbased/commands-v2/subsystems.html)
- [Command scheduler and requirements](https://docs.wpilib.org/en/latest/docs/software/commandbased/commands-v2/command-scheduler.html)

Use the imports supported by this project's installed WPILib version.

## Logging and IO layers (AdvantageKit)

The robot uses [AdvantageKit](https://docs.advantagekit.org/) so every match can
be replayed. `LoggingConstants.CURRENT_MODE` is `REAL` on the roboRIO and
`SIM_MODE` (`SIM` or `REPLAY`) on the desktop.

Each `<Name>IO.java` holds everything hardware-facing for that subsystem:

- `<Name>IOInputs` — every sensor value, with hand-written `toLog`/`fromLog`.
  When you add a field, add it to both; `LoggableInputsTest` fails if a field is
  not logged.
- the `<Name>IO` interface — empty default methods, used as-is for replay.
- `<Name>IO.Real` — the real devices (`VisionIO.Limelight` for vision).
- `<Name>IO.Sim` — extends `Real` and feeds simulated physics into the devices.

- Hardware lives only in IO implementations. A subsystem never constructs a
  `TalonFX`, `CANcoder`, or Limelight helper call directly; it holds an `IO`
  passed in by `RobotContainer`.
- Each subsystem's `periodic()` calls `io.updateInputs(inputs)` and then
  `Logger.processInputs("<Name>", inputs)` before anything reads a sensor.
  Status methods (`atState`, `getPose`, ...) read from `inputs`, never from
  hardware, so replay produces the same decisions.
- Every value the code decides (applied state, setpoint, accepted vision pose,
  readiness) is recorded with `Logger.recordOutput`. `StateHandler.log()` records
  all desired/current states each loop, and `Robot` records active commands.
- To replay a log, set `SIM_MODE = Mode.REPLAY`, run the simulator, and pick the
  `.wpilog`; the recomputed outputs are written to a `_sim` log for AdvantageScope.

## Subsystem contract

Each mechanism subsystem owns, through its IO layer:

- every motor controller, encoder, beam break, range sensor, and other device
  physically attached to that mechanism;
- its state enums and the control request/setpoint represented by each state;
- raw measurements and meaningful status methods such as `hasPiece()`,
  `atState(...)`, or `readyToShoot()`;
- hardware application methods used by its state machine;
- simple one-subsystem command factories.

Construct every hardware device exactly once. Do not access another subsystem's
private motor or encoder. Do not duplicate readiness math at call sites; expose
one status method from the owning subsystem.

Each subsystem has a nested class in `Constants.java`, for example
`IntakeConstants`, `RollersConstants`, `ShooterConstants`, `VisionConstants`,
and `SwerveConstants`. It contains IDs, bus names, inversions, ratios, limits,
gains, tolerances, geometry, and the distance-based lookups in
`InterpolationConstants`. Runtime state, subsystem instances, and commands do not
belong in `Constants`.

Per-state setpoints do **not** live in `Constants`. Each state enum entry carries
its own Phoenix control request (`PositionVoltage`, `VelocityVoltage`,
`VoltageOut`, `DutyCycleOut`, …) with the numeric setpoint written inline in the
request, for example `INTAKE(new VoltageOut(8.5))` or
`SHOOTING(rpmRequest(0))`. `applyState(...)` then does
`io.set…Control(state.request)` and logs the state and setpoint, and `atState(...)`
reads the target back off the same request. Distance-varying setpoints (drum RPM, hood angle) come from the
`InterpolationConstants` maps through Vision, not from a fixed enum value.

## State-machine contract

Install each state machine as the default command of the subsystem whose
hardware it controls. A state machine:

1. reads the `StateHandler` desired states into local `desired…State` variables;
2. applies local ordering, collision, sensor, and jam guards by adjusting those
   local variables (never the `StateHandler` field);
3. sends the local desired states to the subsystem; and
4. publishes current states only after the subsystem confirms them.

Logic based only on one mechanism stays in its state machine. Logic that
coordinates mechanisms belongs in `commands/`. A hard damage-prevention guard
may also be enforced by the state machine as a final safety layer.

## Shared state

`StateHandler` is limited to facts that multiple components need:

- desired and current mechanism states;
- persistent operator or autonomous selections;
- debounced game-piece facts when multiple components consume them;
- alliance and other truly robot-wide modes.

Ownership is strict:

- subsystem command factories write desired mechanism states;
- the matching state machine writes current mechanism states;
- the hardware-owning subsystem writes sensor facts;
- consumers may read those values but must not become additional writers.

`StateHandler` is a singleton — reach it with `StateHandler.getInstance()`, never
by passing it through a constructor. The same holds for other singleton
subsystems (`SwerveSubsystem.getInstance()`). If a class touches the singleton
only once, call `StateHandler.getInstance().field` inline at that spot; if it
touches it more than once, cache it in a field
(`private final StateHandler stateHandler = StateHandler.getInstance();`) and use
that field throughout the class.

Update alliance once before the scheduler runs:

```java
stateHandler.isRedAlliance =
    DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
CommandScheduler.getInstance().run();
```

The swerve pose estimator is the pose source of truth. Expose the live pose from
`VisionSubsystem.getPose()` for field calculations; do not maintain a second
pose cache.

## Vision and swerve

- Vision sends robot orientation to every configured Limelight, validates each
  MegaTag2 estimate, assigns distance-based uncertainty, and sends accepted
  measurements to swerve odometry.
- Swerve owns odometry, alliance perspective, PathPlanner configuration,
  characterization, and the generated CTRE drivetrain behavior.
- Field coordinates use WPILib's blue-origin convention. Apply an alliance
  transform exactly once.
- Hardware values copied from another robot must remain clearly marked until
  measured on this robot.

Autonomous organization is intentionally outside the current implementation
scope.

## Definition of done

Before considering a feature complete, verify:

- IDs, ratios, inversions, limits, gains, units, and tolerances are not
  placeholders;
- each hardware device has one owner;
- desired/current fields have one writer each;
- all enum states are handled;
- command cancellation leaves a safe state;
- state transitions and sensor failure behavior are testable;
- `./gradlew test` or `./gradlew build` passes; and
- remaining hardware assumptions are documented in `ROBOT_OVERVIEW.md`.
