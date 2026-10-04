# Match log workflow

One USB stick carries everything from the robot and Driver Station to the analysis computer.

```text
match ──> roboRIO writes AdvantageKit logs to the stick (/U/logs)
      ──> stick into the DS laptop: tools/collect-ds-logs.sh adds the DS logs (dslogs/)
      ──> stick into the analysis computer: analyze.py ingest --report
      ──> Claude Code: /analyze-wpilog report / diagnose
```

## Stick setup (once)

- FAT32 (or exFAT) USB stick that stays in the roboRIO's USB port during matches.
- Nothing else needed: robot code creates `logs/` on it automatically. Without a stick, the robot
  saves no logs at all.

## After each match (drive team, ~1 minute)

1. **Power the robot off** before pulling the stick. Pulling it while the code runs can cut off
   the end of the current log.
2. Plug the stick into the **Driver Station laptop** and run the collector:
   - double-click `tools\collect-ds-logs.cmd`, or
   - in Git Bash from the repo: `tools/collect-ds-logs.sh`
   It finds the stick by its `logs/` folder, copies the last 2 days of DS logs into `dslogs/`, and
   prints how many robot logs are on the stick. Use `--days 7` for older ones, or pass the drive
   (`tools/collect-ds-logs.sh E:`) if it can't find it. Running it again only copies what changed.
3. Put the stick back in the roboRIO before the next match.

If the warning says there are **no AdvantageKit logs** on the stick, it wasn't in the roboRIO (or
the code didn't run). The DS logs still get copied and can be read on their own.

## On the analysis computer

Needs this repo and Python 3.8+ (nothing to install). With the stick plugged in, from the repo:

```bash
python .claude/skills/analyze-wpilog/scripts/analyze.py ingest --report
```

That:
- copies every robot log off the stick into `logs/matches/<log name>/` (gitignored), skipping ones
  already copied;
- matches each robot log with the Driver Station log recorded at the same time (using the real
  clock time inside both files) and copies it into the same folder;
- writes `report.md` in each new match folder, and `logs/matches/INDEX.md` listing every match
  with its critical/warning counts. Logs with under 5 s enabled (pit restarts, redeploys) are
  copied but not reported.
- DS logs with no matching robot log go to `logs/matches/_ds_only/`.

Nothing on the stick is ever deleted. Clear it yourself when it gets full (copy first).

Then in Claude Code you can just say things like:
- "ingest the stick and tell me how today's matches went"
- "the hood didn't move in q12, why?"
- "fix it" (for a code cause; it verifies with a replay of that match)

## What the DS logs add

The robot log can't see what happens outside the roboRIO. The DS log adds:
- radio link quality: packet loss, round-trip time, and lost-communication events
- controller unplugs and DS laptop lag
- power distribution current for every breaker channel. Name the channels in
  `PD_CHANNELS` in `.claude/skills/analyze-wpilog/scripts/robot_profile.py` so reports say
  "Drum leader" instead of "channel 10".

## Troubleshooting

| Problem | Fix |
|---|---|
| Collector: "No Driver Station logs found" | Run it on the DS laptop; logs live in `C:\Users\Public\Documents\FRC\Log Files\DSLogs` |
| Collector: "Couldn't find the robot's log stick" | Pass the drive letter: `tools/collect-ds-logs.sh E:` |
| `collect-ds-logs.cmd`: "Git Bash not found" | Install Git for Windows |
| Ingest: DS log column shows `-` | That robot log never got a real clock (DS never connected), or the DS log wasn't collected |
| A report says "no valid clock" | Same: the roboRIO clock is only set once the DS connects |
