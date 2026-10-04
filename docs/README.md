# 2026 Rebuilt Offseason 9470 Rebuilt documentation

1. [`ROBOT_OVERVIEW.md`](ROBOT_OVERVIEW.md) — the robot's mechanisms, game-piece
   path, motor inventory, and unresolved hardware decisions.
2. [`CODEBASE_ARCHITECTURE_PLAYBOOK.md`](CODEBASE_ARCHITECTURE_PLAYBOOK.md) —
   the coding rules every change must follow.
3. [`IMPLEMENTATION_GUIDE.md`](IMPLEMENTATION_GUIDE.md) — current source status,
   implementation order, and examples.
4. [`SIMULATION_AND_LOGGING.md`](SIMULATION_AND_LOGGING.md) — running the
   simulator, choosing simulate or replay, AdvantageScope, and robot logs.
5. [`LOG_ANALYSIS_WITH_CLAUDE.md`](LOG_ANALYSIS_WITH_CLAUDE.md) — after each match:
   collect DS logs onto the robot's USB stick, then have Claude Code analyze everything; also
   how to test the whole thing with the simulator.
6. [`../.claude/skills/analyze-wpilog/SKILL.md`](../.claude/skills/analyze-wpilog/SKILL.md) —
   the Claude Code skill for log reports, root-cause diagnosis, and log-verified
   fixes. In Claude Code: `/analyze-wpilog report`, `/analyze-wpilog diagnose
   <what went wrong>`, `/analyze-wpilog fix`. The scripts also run on their own:
   `python .claude/skills/analyze-wpilog/scripts/analyze.py --help`.

When the documents disagree, use `ROBOT_OVERVIEW.md` for the physical design,
the architecture playbook for ownership and organization, and the current code
plus test results for implemented behavior. Update the appropriate document
when a hardware or architectural decision changes.
