#!/usr/bin/env bash
# Copies Driver Station logs (.dslog + .dsevents) onto the robot's log USB stick, next to the
# AdvantageKit logs, so one stick carries everything to the analysis computer.
#
# Run on the Driver Station laptop after pulling the stick out of the roboRIO (robot powered off):
#   tools/collect-ds-logs.sh              find the stick automatically
#   tools/collect-ds-logs.sh E:           or name it (E:, /e, /Volumes/ROBOT, ...)
#   tools/collect-ds-logs.sh --days 7     how far back to copy DS logs (default 2 days)
#   tools/collect-ds-logs.sh --all        copy every DS log on the laptop
#
# On Windows, run it in Git Bash, or double-click tools/collect-ds-logs.cmd.
# Never deletes anything. Files already on the stick are only re-copied if the laptop's copy is
# newer, so it is safe to run after every match.

set -euo pipefail

DAYS=2
ALL=0
USB=""

while [ $# -gt 0 ]; do
  case "$1" in
    --days) DAYS="$2"; shift 2 ;;
    --all) ALL=1; shift ;;
    -h|--help) sed -n '2,15p' "$0"; exit 0 ;;
    *) USB="$1"; shift ;;
  esac
done

is_windows() { command -v cygpath >/dev/null 2>&1; }

to_unix_path() {
  # "E:" / "E:\" / "E:/logs" -> /e, /e/logs in Git Bash; other paths unchanged.
  if is_windows; then cygpath -u "$1"; else printf '%s\n' "$1"; fi
}

# --- Where the Driver Station keeps its logs -------------------------------------------------
DS_DIR=""
if is_windows; then
  PUBLIC_DIR="$(cygpath -u "${PUBLIC:-C:\\Users\\Public}")"
  # DS 2025+ writes into DSLogs/; older versions write directly into Log Files/.
  for d in "$PUBLIC_DIR/Documents/FRC/Log Files/DSLogs" "$PUBLIC_DIR/Documents/FRC/Log Files"; do
    if ls "$d"/*.dslog >/dev/null 2>&1; then DS_DIR="$d"; break; fi
  done
fi
if [ -z "$DS_DIR" ]; then
  echo "No Driver Station logs found (expected C:\\Users\\Public\\Documents\\FRC\\Log Files\\DSLogs)."
  echo "Run this on the Driver Station laptop."
  exit 1
fi

# --- Find the robot's USB stick --------------------------------------------------------------
# The roboRIO writes AdvantageKit logs to /U/logs, so the stick has a logs/ folder at its root.
looks_like_stick() { [ -d "$1/logs" ] || [ -d "$1/dslogs" ]; }

if [ -n "$USB" ]; then
  USB="$(to_unix_path "$USB")"
  USB="${USB%/}"
  if [ ! -d "$USB" ]; then echo "Not found: $USB"; exit 1; fi
else
  candidates=()
  if is_windows; then
    for d in /d /e /f /g /h /i /j /k /l /m /n /o /p /q /r /s /t /u /v /w /x /y /z; do
      if [ -d "$d" ] && looks_like_stick "$d"; then candidates+=("$d"); fi
    done
  else
    for d in /Volumes/* /media/"$USER"/* /run/media/"$USER"/*; do
      if [ -d "$d" ] && looks_like_stick "$d"; then candidates+=("$d"); fi
    done
  fi
  if [ ${#candidates[@]} -eq 0 ]; then
    echo "Couldn't find the robot's log stick (a drive with a logs/ folder)."
    echo "Plug it in, or pass its drive letter: tools/collect-ds-logs.sh E:"
    exit 1
  fi
  if [ ${#candidates[@]} -gt 1 ]; then
    echo "More than one possible stick: ${candidates[*]}"
    echo "Pass the right one, e.g. tools/collect-ds-logs.sh ${candidates[0]}"
    exit 1
  fi
  USB="${candidates[0]}"
fi

DEST="$USB/dslogs"
mkdir -p "$DEST"

# --- Copy --------------------------------------------------------------------------------------
if [ "$ALL" -eq 1 ]; then
  age=()
  echo "Copying every DS log from $DS_DIR"
else
  age=(-mtime "-$DAYS")
  echo "Copying DS logs from the last $DAYS day(s) from $DS_DIR"
fi

copied=0
skipped=0
while IFS= read -r -d '' f; do
  target="$DEST/$(basename "$f")"
  if [ -e "$target" ] && [ ! "$f" -nt "$target" ]; then
    skipped=$((skipped + 1))
  else
    cp -p "$f" "$target"
    copied=$((copied + 1))
  fi
done < <(find "$DS_DIR" -maxdepth 1 -type f \( -name '*.dslog' -o -name '*.dsevents' \) "${age[@]}" -print0)

akit_count=$(find "$USB/logs" -maxdepth 1 -name '*.wpilog' 2>/dev/null | wc -l | tr -d ' ')
newest=$(ls -t "$USB/logs"/*.wpilog 2>/dev/null | head -1 || true)

echo
echo "Stick:            $USB"
echo "DS files copied:  $copied (already up to date: $skipped)"
echo "AdvantageKit logs on the stick: $akit_count"
if [ -n "$newest" ]; then echo "Newest robot log: $(basename "$newest")"; fi
if [ "$akit_count" -eq 0 ]; then
  echo "WARNING: no AdvantageKit logs on this stick. Was it in the roboRIO during the match?"
fi
df -h "$USB" 2>/dev/null | tail -1 | awk '{print "Space left on stick: " $4}' || true
echo
echo "Done. Eject the stick safely, then on the analysis computer run:"
echo "  python .claude/skills/analyze-wpilog/scripts/analyze.py ingest --report"
