# Diagnosis guide: mechanical vs electrical vs programming

Use the log to decide which layer failed. Each layer leaves a different fingerprint in the signals.
Always look at the **same seconds** across signals: setpoint, measured, applied volts, stator and
supply current, Connected, battery, and the state/command timeline.

## The fingerprints

| What the log shows | Points to | Typical real cause |
|---|---|---|
| `Connected` drops; several devices on one bus drop together | Electrical | loose CAN connector, broken CAN wire, missing terminator; devices after the break all drop |
| One device drops alone, sometimes with a current spike just before | Electrical | that controller's power (breaker, PDH slot, Anderson connector) or its CAN pigtail |
| Battery under ~8 V when motors start; `BrownedOut` true | Electrical | weak/old battery, loose main lead or breaker, everything accelerating at once |
| `CommsDisableCount` increases | Electrical | radio power, radio-roboRIO ethernet, field network |
| CAN utilization over 70%, bus-off or TX-full counts climbing | Electrical | wiring fault or bus overload (too many status signals) |
| High applied volts + high stator current + no motion | Mechanical | jam, binding, game piece stuck, hitting a hard stop |
| Motion but slower than the setpoint, sitting at the stator limit | Mechanical (or limit too low) | friction, tight belt/chain, rubbing, heavy load |
| Current rises over the match on the same move; temperature climbs | Mechanical | something getting tighter (bearing, rubbing); also motors sharing load unevenly |
| One drum/swerve motor draws far more current than its siblings | Mechanical or Programming | binding on that stage, **or** a follower aligned the wrong way (fighting) |
| Position past the soft limits | Mechanical (zero lost) or Programming (zero/offset wrong) | not homed, belt skipped, encoder offset wrong |
| Desired state never requested | Programming | binding missing or command not running (check `Commands/*`) |
| Applied state differs from desired | Programming (sometimes intended) | state machine guard; check it is the guard you expect |
| Setpoint 0 for an active state | Programming | placeholder value in the state enum, or empty interpolation map |
| Setpoint outside the soft limits | Programming | the target can never be reached; clamp it or fix the map/enum |
| Far from the setpoint but ~0 V applied | Programming | soft limit blocking, neutral/coast request, wrong request type, motor disabled |
| Moves the opposite way to the command | Programming / Config | `InvertedValue`, follower `MotorAlignmentValue`, or sensor direction |
| Reaches the target but the current state never confirms | Programming | tolerance too tight, at-state compares the wrong units |
| A desired state flips back and forth every loop | Programming | two commands writing the same `StateHandler` field |
| Oscillates around the target | Tuning | kP too high, missing kD, Motion Magic too aggressive |
| Settles short of the target with low output | Tuning | kP too low, missing kS (friction) or kG (gravity / gravity offset) |
| Overshoots then settles | Tuning | kP too high, profile accel too high, no kD |

## How to tell look-alikes apart

**Stall: jam vs hard stop vs soft limit.** All three hold position with the setpoint unreached.
- Soft limit: applied volts drop to ~0 at the limit and the measured value sits right at it.
- Hard stop: volts and current stay high, and the measured value is at the mechanism's travel end
  (compare with the soft limits; if the hard stop is *inside* them, the limits or zero are wrong).
- Jam: volts and current high somewhere in the middle of travel, often only with a game piece
  (check whether intake/feed commands were running).

**Sagging arm: gravity config vs weak motor vs friction.** If the error depends on the angle (worst
near level, small near vertical) it is kG or the gravity offset. If it is the same at every angle,
think kS/friction. If it gets worse through the match, think heat or mechanical wear.

**Drum slow to spin up: follower fighting vs battery vs gains.** Fighting: one motor's current far
above the others from the first second. Battery: all motors' current fine but volts limited by a
sagging battery. Gains: currents normal, volts below 11 V, error shrinking slowly (kV too low).

**Disconnect: wiring vs brownout.** If `BrownedOut` or a battery dip is at the same moment,
the controller rebooted from low voltage (electrical, but the fix is the battery or current draw).
If the battery was fine, it is wiring or that controller's power.

**"Shooter never fed".** Walk `readyToShoot`: was a shoot command running? Drum setpoint above 0?
Drum within tolerance? Hood within tolerance? For ranged shots, heading within 5 degrees? The
report's "not ready because" lines give the time spent blocked by each. A 0 setpoint is
programming (empty interpolation map). Close-but-never-within-tolerance is tuning or a tolerance
choice. Far away with high current is mechanical.

## Confirming without the log

Suggest the cheapest test that separates the remaining hypotheses:
- Robot off: move the mechanism by hand through full travel. Feel for tight spots, check belts and
  set screws.
- Phoenix Tuner X with the robot code not running: command the motor at low voltage, watch
  position, velocity, current, and faults. Bypasses all the robot code.
- Swap the battery and repeat. Watch the PDH channel current in Tuner X / AdvantageScope.
- Replay the log with a candidate code change (`scripts/replay.py`) to prove a decision-level fix.

## Confidence

- **High:** two or more independent signals agree, and the timing lines up (cause before effect).
- **Medium:** one strong signal, with alternatives not ruled out.
- **Low:** only the absence of other explanations. Say what test would raise it.
