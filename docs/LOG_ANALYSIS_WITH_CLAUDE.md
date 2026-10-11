# Robot logs with Claude Code

How to get from "we just played a match" to "here's what went wrong and the fix". You don't need
to know how any of it works inside; follow the steps and ask Claude in plain English.

## One-time setup

| Where | What |
|---|---|
| Robot | A USB stick (FAT32 or exFAT) left in the roboRIO's USB port. No stick = no robot logs. |
| Driver Station laptop | This repo (or at least its `tools/` folder) and [Git for Windows](https://git-scm.com/download/win). |
| Analysis computer | This repo, Python 3.8 or newer, and Claude Code opened in the repo folder. |
| Optional | Fill in the **Power distribution channels** table in `docs/ROBOT_OVERVIEW.md` so reports name each breaker channel ("Drum leader: 58 A" instead of "channel 10: 58 A"). |

## After every match (drive team, about a minute)

1. **Power the robot off**, then take the USB stick out of the roboRIO.
2. Plug the stick into the **Driver Station laptop** and double-click **`tools\collect-ds-logs.cmd`**.
   It adds the most recent match's Driver Station log to the stick and tells you how many robot
   logs are on it.
   - Missed running it after some matches? Grab more from Git Bash:
     `tools/collect-ds-logs.sh --last 3` (newest 3), `--days 1` (today), or `--all`.
   - "The newest DS log started ... min away from the newest robot log": the robot was turned on
     again after the match, so the newest DS log is a pit session. Run it with `--last 3`.
   - "Couldn't find the robot's log stick": run it from Git Bash with the drive letter:
     `tools/collect-ds-logs.sh E:`
   - "No AdvantageKit logs on this stick": the stick wasn't in the robot, or the robot code
     didn't run. The DS logs still get copied.
3. Put the stick back in the roboRIO before the next match.

Run step 2 after every match (by default it only grabs the latest match). Running it twice is
harmless.

## On the analysis computer

Plug the stick in, open Claude Code in this repo, and say:

> **ingest the stick and tell me how today's matches went**

Claude copies everything off the stick (nothing on the stick is changed or deleted), writes a
report for every match, and gives you a summary: what went wrong, when, and whether it looks
mechanical, electrical, or a code problem.

Then just ask follow-up questions:

| You want | Say something like |
|---|---|
| Details on one match | "go over q14 in detail" |
| Why something happened | "the hood didn't move in q12, why?" / "we lost comms at the end of q9, what caused it?" |
| A cause you can act on | "is that mechanical or code?" / "what should we check in the pit?" |
| A code fix | "fix it" (Claude explains the change first, then makes it on a branch and checks it against that match's log) |
| Trends | "is the drum getting hotter across matches?" / "which matches had brownouts?" |
| One log someone sent you | "analyze `C:\Users\me\Downloads\akit_26-10-03_18-02-11_njfla_q12.wpilog`" |

The reports are in `logs/matches/<match>/report.md`, with an overview in
`logs/matches/INDEX.md`. The `logs/` folder is never committed.

Times in reports (like `t=83.4s`) are seconds from the start of that log, so you can jump straight
to them in AdvantageScope.

## Test it with the simulator

Use this to check everything works before an event, no robot needed.

1. In the WPILib VS Code: `Ctrl+Shift+P` → **WPILib: Simulate Robot Code**.
2. In the popup check **Sim GUI**. To also test Driver Station logs, check **DS Socket** too and
   open the real **FRC Driver Station** app; it connects to the simulator on this computer (if it
   doesn't, set its team number to `127.0.0.1`).
3. In SmartDashboard > **Robot Startup**, leave Run Mode on **Simulate** and set **Start** to true
   (see `docs/SIMULATION_AND_LOGGING.md` if you haven't done this before).
4. **Enable** in Teleop (in the Driver Station app, or the Sim GUI's Robot State window), drive
   around and press some buttons for 30 seconds or so, then **disable**.
5. Close the simulator window so the log is finished.
6. Tell Claude:

   > **test the log workflow with my latest sim log**

Claude builds a pretend USB stick from that sim log, runs the same DS collector script the drive
team uses, ingests it, and prints a checklist:

```text
PASS  log decoded
PASS  enabled time            <- you enabled the robot in the sim
PASS  mechanisms found from the log
PASS  Constants.java read for tolerances/limits
PASS  Driver Station log paired   <- only if you used the real DS app (step 2)
PASS  report written
PASS  index written
```

(Without Claude: `python .claude/skills/analyze-wpilog/scripts/analyze.py simtest`.)

In the simulator the battery, currents, and temperatures are fake, and mechanisms with 0
setpoints don't move, so the report's electrical and mechanical findings mean nothing there. The
test checks that logs flow through correctly.

## If something looks off

| Problem | What to do |
|---|---|
| A match has no DS log in the index | The collector wasn't run right after that match (rerun it with `--last N` or `--days 1`), or the robot never connected to the DS (so its clock was never set). |
| The report says it couldn't find a tolerance or limit | Someone renamed something in `Constants.java`; tell Claude "the report says it couldn't find X" and it will sort it out. |
| A new mechanism doesn't show up in reports | It needs to log like the others (a `Setpoint...` output and the usual inputs). Ask Claude: "why isn't the new mechanism in the report?" |
| `collect-ds-logs.cmd` says Git Bash not found | Install Git for Windows. |
| Claude can't find the stick | Tell it the drive letter: "ingest the stick on E:". |
