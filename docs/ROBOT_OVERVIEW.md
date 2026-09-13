# 2026 Rebuilt 9470 Offseason Robot Overview

## Robot concept

This robot is planned as an FRC 9470-style clone built from the team's `Rebuilt_2026`
software and design experience. The largest change from last year's robot is the
shooter: this robot uses a drum shooter and adjustable hood instead of a turret-style
shooter. The planned software is divided into five subsystems: Shooter, Intake,
Rollers, Vision, and Swerve.

The intended game-piece path is:

`Intake rollers -> hopper floor roller -> extendable hopper -> tunnel/ramp -> shooter drum`

## Mechanisms and subsystems

### Shooter

The Shooter subsystem controls the four Kraken X60 motors that drive the shooter drum
and the Kraken X44 that pivots the hood. The drum provides launch speed, while the hood
changes the launch angle. Because there is no turret, horizontal aiming will be handled
by the robot's heading through the Swerve subsystem.

Planned responsibilities include drum velocity control, hood position control,
at-speed/at-angle readiness checks, safe idle behavior, and coordinated feeding from
the tunnel.

### Intake

The Intake subsystem controls one Kraken X60 for the intake pivot and two Kraken X60s
for the intake rollers. It will deploy and retract the intake, pull game pieces into the
robot, stop or reverse to clear jams, and hand game pieces to the hopper.

### Rollers

The Rollers subsystem moves game pieces through the robot after collection. It includes
one Kraken X60 for the floor roller inside the hopper and two Kraken X60s for the
tunnel/ramp. It will coordinate hopper storage, indexing, feeding into the shooter, and
reversing to clear jams.

The hopper is extendable. Its extension actuator and sensing have not yet been specified,
so they are not included in the motor count below. Once selected, hopper extension and
retraction should be owned by this subsystem unless the mechanism becomes complex enough
to justify a separate subsystem.

### Vision

The Vision subsystem uses four Limelights for AprilTag localization. All four cameras
independently feed MegaTag2 estimates into the swerve pose estimator. Measurements are
rejected while the robot is rotating very quickly, and estimates become less trusted as
tag distance grows.

The NetworkTables names are `limelight-front-left`, `limelight-front-right`,
`limelight-rear-left`, and `limelight-rear-right`. The robot-to-camera poses must still
be measured and configured on the physical robot.

### Swerve

Swerve has been copied from Rebuilt_2026 codebase. It provides field-centric driving, robot-centric support, odometry, vision fusion. PathPlanner integration, simulation, and characterization commands. Its hardware IDs,
encoder offsets, gear ratios, wheel radius, and PID/feedforward gains are copied values
and must be checked before the new robot is enabled (TODO: retune swerve to get new TunerConstants).

## Specified motor inventory

| Mechanism | Motor | Quantity |
| --- | --- | ---: |
| Shooter drum | Kraken X60 | 4 |
| Hood pivot | Kraken X44 | 1 |
| Intake pivot | Kraken X60 | 1 |
| Intake rollers | Kraken X60 | 2 |
| Hopper floor roller | Kraken X60 | 1 |
| Tunnel/ramp | Kraken X60 | 2 |
| **Specified mechanism total** |  | **11** |

This total does not include the swerve drive motors or an actuator for the extendable
hopper.

## Coordination goals

- Intake deploys and collects while the hopper and tunnel accept game pieces.
- Rollers stop or index game pieces so the shooter is not fed early.
- Vision and swerve provide robot pose and heading for fixed-shooter aiming.
- Shooter declares readiness only when drum speed and hood angle are within tolerance.
- Feeding begins only after the shooter is ready, then all transport rollers run in a
  coordinated direction.
- Every mechanism needs a safe stopped state plus a deliberate reverse/jam-clear state.

## Information still needed

- Extendable hopper actuator type, motor/controller count, travel limits, and sensors.
- Final CAN IDs, motor inversions, gear ratios, current limits, and neutral modes.
- Drum motor grouping and direction (including whether the four motors are mechanically
  linked or control multiple drum stages).
- Hood absolute-position sensor, usable angle range, and zeroing method.
- Game-piece sensors in the intake, hopper, tunnel, and shooter throat.
- Final measured mounting pose for each of the four Limelights.
- Confirmation that the copied swerve module hardware and calibration are unchanged.
