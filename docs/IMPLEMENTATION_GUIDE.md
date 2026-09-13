# 2026 Rebuilt 9470 Offseason implementation guide

This guide translates [`ROBOT_OVERVIEW.md`](ROBOT_OVERVIEW.md) into code. Follow
the rules in [`CODEBASE_ARCHITECTURE_PLAYBOOK.md`](CODEBASE_ARCHITECTURE_PLAYBOOK.md).

## Robot being implemented

The robot has five intended subsystems:

| Subsystem | Hardware and responsibility |
|---|---|
| `ShooterSubsystem` | Four Kraken X60 drum motors and one Kraken X44 hood motor; launch speed, hood angle, and readiness |
| `IntakeSubsystem` | One pivot motor and two roller motors; deploy, collect, retract, and reverse |
| `RollersSubsystem` | Hopper floor roller, two tunnel/ramp motors, and eventually hopper extension; store, index, feed, and clear jams |
| `VisionSubsystem` | Four Limelights; MegaTag2 localization and field calculations |
| `SwerveSubsystem` | CTRE drivetrain; driving, heading, odometry, and PathPlanner |

Game pieces travel through:

```text
intake -> hopper floor -> extendable hopper -> tunnel/ramp -> shooter drum
```

Because this shooter has no turret, `SwerveSubsystem` aims the robot heading.
Feeding may begin only when the shooter drum, hood, and robot heading are ready.

## Current code status

Treat the source as a partial scaffold, not a completed implementation:

| Area | Current status | Required next work |
|---|---|---|
| `RobotContainer` | Constructs every subsystem, installs all default state machines, and holds a driver and operator controller | Fill the Driver Bindings and Operator Bindings sections after the control map is specified |
| `StateHandler` | Alliance, swerve, Intake, Rollers, and Shooter state are implemented | Add only genuinely shared sensor/selection facts as needed |
| `IntakeSubsystem` | Intent factories, hardware API, state machine, and guard tests exist | Replace placeholder hardware values and add game-piece sensors |
| `RollersSubsystem` | Floor-roller intent factories and default state machine exist | Add the tunnel/ramp and hopper extension when their hardware is specified; replace the zero RPM requests in `RollerFloorState` |
| `ShooterSubsystem` | Drum/hood API, factories, readiness, and state machine exist | Verify branch-derived IDs/directions, configure gains, fill the `InterpolationConstants` maps, and drive tracking setpoints from Vision |
| `VisionSubsystem` | Four-camera MT2 fusion, live `getPose()`, hub-distance, and shooter map queries (`getRPM()`, `getHoodAngle()`) exist | Add measured camera transforms, logging/tests, and tune rejection uncertainty |
| `SwerveSubsystem` | Default modes, target-facing mode, odometry, and generated CTRE behavior exist | Verify every copied hardware value and tune heading control |
| `commands/RobotCommands` | Collection, shooting, and ejection compositions exist | Extend them when tunnel/hopper sensing is implemented |

Do not hide unknown hardware values behind plausible guesses. Keep them as
named TODOs and list the missing measurements in `ROBOT_OVERVIEW.md`.
`RobotContainer` constructs every subsystem directly, so unverified IDs will
still try to reach hardware — check every ID against the swerve CAN map and
verify the related constants before deploying to the real robot.

## Recommended implementation order

1. Measure and fill subsystem constants and camera poses.
2. Verify the Intake and Shooter constants already represented in code.
3. Add Intake game-piece sensors and validate the deployed-before-intake guard.
4. Implement and tune the Shooter shot model and heading target supplier.
5. Build the complete Rollers subsystem around floor, tunnel/ramp, and hopper
   extension once its actuator and sensors are known.
6. Validate all four camera transforms and measurement rejection behavior.
7. Add the operator bindings, telemetry, and remaining state-machine tests.

Complete one subsystem vertically—constants through tested behavior—before starting
several half-implemented mechanisms.

## State design

Use separate state enums for independently controlled axes. The initial state
set should stay small and represent physical behavior, not controller buttons.
Each enum entry holds its own Phoenix control request with the setpoint written
inline (for example `INTAKE(new VoltageOut(8.5))`), not a raw number pulled from
`Constants`; the subsystem applies `state.request` directly.

### Intake

```text
IntakeArmState:    STOWED, DEPLOYED, EXTENDED
IntakeRollerState: REST, INTAKE, HOLD, EJECT
```

The intake state machine owns arm-before-roller ordering and local jam rules.
Factories such as `intakeCommand()`, `stowCommand()`, and `ejectCommand()` live
in `IntakeSubsystem`.

### Rollers

```text
FloorState:  REST, INTAKE, FEED, EJECT
TunnelState: REST, INDEX, FEED, EJECT
HopperState: RETRACTED, EXTENDED   // only after hardware is specified
```

The rollers state machine owns sequencing between the floor, tunnel, local
sensors, and jam recovery. It does not decide whether the shooter or swerve is
ready; the coordinated shoot command supplies that permission.

### Shooter

```text
DrumState: REST, SHOOT, REVERSE
HoodState: STOWED, AIMING, FIXED
```

Final names may change when shot modes are defined. `ShooterSubsystem` should
expose `atDrumSpeed()`, `atHoodAngle()`, and one `readyToShoot()` method. Dynamic
setpoints come from the selected target/distance model, not by mutating a shared
enum request object.

## Example: applying the old spindexer lesson to Rollers

The old spindexer/ramp behavior maps to the new `RollersSubsystem`. Its current
implementation owns the floor; add tunnel and hopper axes to the same class as
their hardware is specified. Keep two different values:

```text
desired state -> StateHandler fields, read each cycle into a local
                 desired…State variable the state machine may adjust for safety
current state -> confirmed StateHandler fields
```

### One-subsystem factory

`RollersSubsystem` owns the simple command that requests staging or feeding.
It accepts a generic permission signal and does not depend directly on Shooter,
Vision, or Swerve.

```java
public Command feedWhenAllowedCommand(BooleanSupplier feedAllowed) {
  return Commands.run(() -> {
    if (feedAllowed.getAsBoolean()) {
      requestFeed();
    } else {
      requestStaging();
    }
  }).finallyDo(interrupted -> requestRest());
}

// These helpers are the only writers of the two desired fields. Define the
// exact staging pair after the game-piece sensors and safe directions are known.
private void requestFeed() { /* request validated FEED states */ }
private void requestStaging() { /* request validated hold/index states */ }
private void requestRest() { /* request REST for both stages */ }
```

This intent command deliberately has no Rollers requirement. The default
`RollersStateMachine` must continue running and remain the hardware owner.

### Local state machine

The state machine keeps only Rollers-local rules. For example, do not run the
floor forward into the tunnel until the tunnel is at feed speed:

```java
@Override
public void execute() {
  FloorState desiredFloorState = stateHandler.desiredFloorState;
  TunnelState desiredTunnelState = stateHandler.desiredTunnelState;

  // Adjust the local desired variable for the interlock; never the StateHandler field.
  if (desiredFloorState == FloorState.FEED
      && (desiredTunnelState != TunnelState.FEED
          || !rollers.atTunnelState(TunnelState.FEED))) {
    desiredFloorState = FloorState.REST;
  }

  rollers.applyTunnelState(desiredTunnelState);
  rollers.applyFloorState(desiredFloorState);

  if (rollers.atTunnelState(desiredTunnelState)) {
    stateHandler.currentTunnelState = desiredTunnelState;
  }
  if (rollers.atFloorState(desiredFloorState)) {
    stateHandler.currentFloorState = desiredFloorState;
  }
}
```

Jam current, beam breaks, range sensors, and timed local unjam pulses also
belong here. Notice that the state machine never changes either desired field.

### Coordinated shoot command

Create this in `commands/` because it coordinates Shooter, Swerve, and Rollers.
Vision continues supplying accepted pose measurements to swerve odometry:

```java
public static Command shootCommand(
    ShooterSubsystem shooter,
    SwerveSubsystem swerve,
    RollersSubsystem rollers,
    Supplier<Pose2d> target) {

  BooleanSupplier feedAllowed = () ->
      shooter.readyToShoot()
          && swerve.isFacing(target.get());

  return Commands.parallel(
      shooter.aimAndSpinCommand(target),
      swerve.faceTargetCommand(target),
      rollers.feedWhenAllowedCommand(feedAllowed));
}
```

The permission is reevaluated every scheduler cycle. If drum speed, hood angle,
or heading readiness is lost, the rollers return to their validated staging
behavior immediately. The factory remains reusable in both teleop and
autonomous.

If a guard prevents physical damage rather than merely preventing a poor shot,
also enforce it as a final local desired-state guard in the hardware-owning state
machine.

## Robot lifecycle and shared accessors

Keep `Robot.robotPeriodic()` small and update alliance before running commands:

```java
@Override
public void robotPeriodic() {
  stateHandler.isRedAlliance =
      DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  CommandScheduler.getInstance().run();
}
```

Expose the live swerve pose through Vision. `VisionSubsystem` grabs the swerve
singleton itself (`SwerveSubsystem.getInstance()`) rather than taking it in its
constructor:

```java
public Pose2d getPose() {
  return swerve.getState().Pose;
}
```

This is an accessor, not a second pose variable. Read it once into a local
snapshot when a calculation needs a consistent pose several times.

`RobotContainer` should only construct the graph and bind commands. Subsystems
and state machines take no `StateHandler` argument — they grab the singleton
themselves:

```java
rollers.setDefaultCommand(new RollersStateMachine(rollers));

driver.rightTrigger().whileTrue(
    RobotCommands.shootCommand(shooter, swerve, rollers, targetSupplier));
```

## Vision implementation

The four Limelights are:

- `limelight-front-left`
- `limelight-front-right`
- `limelight-rear-left`
- `limelight-rear-right`

For every camera:

1. configure its measured robot-space pose;
2. send current robot yaw before reading MegaTag2;
3. reject null/no-tag estimates and estimates captured above the gyro-rate
   limit;
4. calculate uncertainty from observed tag geometry/distance;
5. preserve the camera timestamp when adding the measurement; and
6. log acceptance/rejection and uncertainty for tuning.

Fuse cameras independently so one invalid camera does not discard the others.
Vision must not own a separate odometry pose.

## Per-feature handoff checklist

For every new mechanism or coordinated action, leave the next agent:

- the state list and physical meaning of each state;
- constants and which values are still unmeasured;
- desired/current field ownership;
- local state-machine guards;
- public command factories and their safe interruption behavior;
- multi-subsystem commands that consume those factories;
- status/readiness definitions;
- the build/test result.
