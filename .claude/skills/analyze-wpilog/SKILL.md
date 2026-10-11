---
name: analyze-wpilog
description: Analyze AdvantageKit .wpilog logs from the 2026 Offseason CloneBot - generate a post-match/practice health report, find the root cause of a robot problem (mechanical, electrical, or programming), and fix code problems, verifying the fix by replaying the log. Use when the user mentions a log, wpilog, match/practice log, AdvantageScope, brownout, a mechanism misbehaving on the real robot ("hood didn't move", "shooter never fed", "intake stalled"), or asks for a report, root cause, or fix based on robot data.
argument-hint: "report [log] | diagnose <what went wrong> [log] | fix [finding]"
---

# Analyze robot logs (CloneBot)

Three jobs. Pick from the user's words; ask only if it is genuinely unclear.

| User says | Job |
|---|---|
| "generate a report", "how did the match go", "anything wrong in this log" | **Report** |
| "why did X happen", "analyze this issue", "is it mechanical or code" | **Diagnose** |
| "fix it", "fix the code", or a diagnosis that ends in a Programming/Tuning cause | **Fix** |

All tools are in `scripts/` (pure Python 3.8+, no installs). Run from the repo root:

```bash
python .claude/skills/analyze-wpilog/scripts/analyze.py <command> ...
```

| Command | What it gives you |
|---|---|
| `find [dirs]` | `.wpilog` files newest first (repo `logs/`, Downloads, Documents, Desktop) |
| `info LOG [--prefix P]` | match info, enabled periods, every key with type and count |
| `report LOG [--out FILE]` | markdown report: prioritized findings, electrical, mechanisms, shooting, states, swerve, vision, commands, alerts |
| `diagnose LOG [MECH] [--from S --to S]` | evidence + ranked causes for one mechanism (or every mechanism with a problem) |
| `timeline LOG [--only mode,state,command,fault,event] [--filter TEXT] [--from --to]` | what happened, in order |
| `series LOG KEY... [--from --to] [--step 0.05]` | raw values as CSV (Pose2d prints as x y heading) |
| `compare SIMLOG` | after a replay: findings fixed/added by the code change, setpoints and states before vs after |
| `replay.py LOG [--no-build]` | runs the **current code** on the log's inputs, writes `LOG_sim.wpilog` |

Mechanism names for `diagnose`: `intake_arm`, `intake_roller`, `floor_roller`, `hood`, `drum`.
Times are seconds since the log started (the report prints `t=12.34s [teleop]`). Tell the user these
times so they can scrub to them in AdvantageScope.

Before any job, read [reference/codebase-map.md](reference/codebase-map.md) if you have not this
session: it maps every log key to the code that writes it and lists this codebase's gotchas.

## Step 0: get the right log

1. If the user gave a path, use it. Otherwise run `find` and pick the newest that fits what they
   described (match number, "last night"). If several fit, list them and ask.
2. Check what kind of log it is with `info`:
   - **Real robot** (`ProjectName` metadata, real battery values, `RobotMode` REAL): everything applies.
   - **Simulation**: battery, currents, and temperatures are fake, and there is no friction or
     jamming. Do not draw electrical or mechanical conclusions from a sim log; only programming ones.
   - **Replay output (`_sim`)**: contains the original `RealOutputs/` *and* the recomputed
     `ReplayOutputs/`. The analyzer uses the recomputed ones by default (`--outputs replay`); pass
     `--outputs real` to see what the robot actually did.
3. Large logs (a full event) take a few seconds per command. That is normal.

## Report

1. `analyze.py report LOG --out logs/reports/<log name>.md`. The `logs/` folder is gitignored.
2. Read the report, then give the user a short summary in plain language:
   - the 3-6 findings that matter most, each with the time, what it means physically, and what to
     check. Group repeated symptoms of one cause (a brownout and the motors that dropped out right
     after it are one problem).
   - say which findings are probably **not real problems**: placeholder setpoints (all 0 while
     tuning), PathPlanner config errors, disconnect alerts in the first seconds after boot.
3. Point to the report file for the full tables. Offer to diagnose the top issue.

Don't paste the whole report into chat.

## Diagnose (mechanical vs electrical vs programming)

Goal: a cause backed by evidence, the confidence level, and the physical or code check that would
confirm it. Follow [reference/diagnosis-guide.md](reference/diagnosis-guide.md).

1. **Pin down the symptom and the window.** What did the user see, and roughly when? Use
   `timeline --filter <word>` or `timeline --only command,state` to find the moment (for example
   when `ShootGamePieces` started). The window is from a few seconds before to the end of the symptom.
2. **Run** `diagnose LOG <mech> --from A --to B`. It prints evidence, flags, a heuristic ranking of
   Electrical / Mechanical / Programming / Tuning, and the code files to read.
3. **Treat the ranking as a lead, not an answer.** Confirm it:
   - pull the raw signals with `series` (setpoint vs measured, applied volts, stator current, battery,
     Connected) for the exact seconds involved;
   - read the code files listed, especially the state enum setpoints, the state machine guards, and
     the `Constants.java` config (inversion, soft limits, current limits, gear ratio, gains);
   - check whether another mechanism or command explains it (the timeline shows overlaps).
4. **Answer** in this shape:
   - **Cause:** one sentence. **Category:** Mechanical / Electrical / Programming / Tuning (or
     "Programming, made visible by Mechanical" when both hold).
   - **Evidence:** 2-4 bullets with times and numbers from the log.
   - **Confidence:** high / medium / low, and what would change your mind.
   - **Next step:** the physical check for Mechanical/Electrical (be specific: "spin the hood by
     hand with the robot off and feel for a tight spot near 12 degrees"), or the code fix for
     Programming/Tuning (offer to make it).

Never call something mechanical or electrical just because the code "looks right". If the log can't
tell the categories apart, say so and name the test that would (for example: command the mechanism
slowly in Phoenix Tuner X with the code disabled).

## Fix

Only fix causes that live in code: Programming, Tuning, Config. For Mechanical or Electrical causes,
explain the hardware fix instead. Never hide a hardware problem in code (raising a current limit to
push through binding, widening a tolerance so a sagging mechanism "reaches" its state) unless the
user explicitly chooses that, knowing the trade-off.

1. **Explain first:** what you will change, why the log says it is needed, and any risk. For
   anything that changes how the robot moves (gains, limits, inversions, setpoints), get a yes before
   editing.
2. **Edit following the architecture** in [reference/codebase-map.md](reference/codebase-map.md):
   setpoints in the state enums, hardware config in `Constants.java`, guards in the state machine,
   one writer per `StateHandler` field.
3. **Build and test** with the WPILib JDK (system Java is too new for this Gradle):
   - Windows PowerShell: `$env:JAVA_HOME='C:\Users\Public\wpilib\2026\jdk'; .\gradlew.bat build`
   - bash: `JAVA_HOME=/c/Users/Public/wpilib/2026/jdk ./gradlew build`
4. **Verify against the log** when the fix changes decisions (states, setpoints, readiness, commands):
   `python .claude/skills/analyze-wpilog/scripts/replay.py LOG` then `analyze.py compare LOG_sim.wpilog`.
   The finding should move to "Fixed by the code change" with nothing new and unexpected. Replay
   does **not** re-simulate physics (positions and speeds are the recorded ones), so it can't prove
   a gain or mechanical change works. Say that, and say what to watch for on the robot.
5. **Keep the profile true.** If you changed a key name, tolerance, current limit, soft limit, or
   file location, update `scripts/robot_profile.py` in the same change and run
   `python .claude/skills/analyze-wpilog/scripts/test_analyze.py`.
6. **Commit on a branch, not main**, with a message that names the log and finding. Don't push or
   open a PR unless asked.

## Limits to tell the user about when relevant

- Heuristics flag patterns, not certainties. A stall can be a jam, a hard stop, or a soft limit
  pinning the setpoint; the evidence decides.
- Values are change-only logged and sampled at 50 Hz, so spikes shorter than ~20 ms may be missed.
  Phoenix signal logs (hoot files) have higher-rate data if it is ever needed.
- While setpoints and gains are still placeholders (0), most mechanisms are never really
  commanded, so a clean report doesn't mean the mechanisms work.
