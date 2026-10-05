#!/usr/bin/env bash
# Copies Driver Station logs (.dslog + .dsevents) onto the robot's log USB stick, next to the
# AdvantageKit logs, so one stick carries everything to the analysis computer.
#
# Run on the Driver Station laptop after pulling the stick out of the roboRIO (robot powered off):
#   tools/collect-ds-logs.sh              the most recent match's DS log (the newest one)
#   tools/collect-ds-logs.sh E:           name the stick if it isn't found (E:, /e, /Volumes/X)
#   tools/collect-ds-logs.sh --last 3     the newest 3 DS logs (e.g. a match split by a reconnect)
#   tools/collect-ds-logs.sh --days 2     every DS log from the last 2 days
#   tools/collect-ds-logs.sh --all        every DS log on the laptop
#
# On Windows, run it in Git Bash, or double-click tools/collect-ds-logs.cmd.
# Never deletes anything. Files already on the stick are only re-copied if the laptop's copy is
# newer, so it is safe to run after every match.

set -euo pipefail

MODE=last   # last N logs (default 1 = the most recent match), or days, or all
LAST=1
DAYS=""
USB=""

while [ $# -gt 0 ]; do
  case "$1" in
    --last) MODE=last; LAST="$2"; shift 2 ;;
    --days) MODE=days; DAYS="$2"; shift 2 ;;
    --all) MODE=all; shift ;;
    -h|--help) sed -n '2,16p' "$0"; exit 0 ;;
    -*) echo "Unknown option $1 (see --help)"; exit 1 ;;
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

# --- Pick which DS logs ------------------------------------------------------------------------
# DS log names start with their local start time ("2026_10_03 18_02_11 Sat.dslog"), so sorting by
# name sorts by time. Each log has a .dsevents file with the same name.
case "$MODE" in
  all)  echo "Copying every DS log from $DS_DIR"
        stems=$(find "$DS_DIR" -maxdepth 1 -type f -name '*.dslog' | sort -r) ;;
  days) echo "Copying DS logs from the last $DAYS day(s) from $DS_DIR"
        stems=$(find "$DS_DIR" -maxdepth 1 -type f -name '*.dslog' -mtime "-$DAYS" | sort -r) ;;
  last) if [ "$LAST" -eq 1 ]; then echo "Copying the most recent match's DS log from $DS_DIR"
        else echo "Copying the newest $LAST DS logs from $DS_DIR"; fi
        stems=$(find "$DS_DIR" -maxdepth 1 -type f -name '*.dslog' | sort -r | head -n "$LAST") ;;
esac

# --- Copy --------------------------------------------------------------------------------------
copied=0
skipped=0
newest_ds=""
while IFS= read -r dslog; do
  [ -n "$dslog" ] || continue
  [ -n "$newest_ds" ] || newest_ds="$dslog"
  for f in "$dslog" "${dslog%.dslog}.dsevents"; do
    [ -e "$f" ] || continue
    target="$DEST/$(basename "$f")"
    if [ -e "$target" ] && [ ! "$f" -nt "$target" ]; then
      skipped=$((skipped + 1))
    else
      cp -p "$f" "$target"
      copied=$((copied + 1))
    fi
  done
done <<< "$stems"

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

# --- Sanity check: does the newest DS log line up with the newest robot log? -------------------
# Robot log names hold the roboRIO's clock (UTC): akit_26-10-03_22-02-11_....wpilog. DS log names
# hold this laptop's local time. A big gap usually means the DS made another log after the match
# (robot turned on again in the pits), so the newest DS log isn't the match.
if [ "$MODE" = last ] && [ -n "$newest_ds" ]; then
  newest_named=$(ls "$USB/logs"/akit_[0-9][0-9]-*.wpilog 2>/dev/null | sort | tail -1 || true)
  ds_name=$(basename "$newest_ds")
  if [ -n "$newest_named" ]; then
    r=$(basename "$newest_named" | sed -E 's/^akit_([0-9]{2})-([0-9]{2})-([0-9]{2})_([0-9]{2})-([0-9]{2})-([0-9]{2}).*/20\1-\2-\3 \4:\5:\6/')
    d=$(printf '%s' "$ds_name" | sed -E 's/^([0-9]{4})_([0-9]{2})_([0-9]{2}) ([0-9]{2})_([0-9]{2})_([0-9]{2}).*/\1-\2-\3 \4:\5:\6/')
    robot_s=$(date -d "$r UTC" +%s 2>/dev/null || true)
    ds_s=$(date -d "$d" +%s 2>/dev/null || true)
    if [ -n "$robot_s" ] && [ -n "$ds_s" ]; then
      gap=$(( robot_s > ds_s ? robot_s - ds_s : ds_s - robot_s ))
      if [ "$gap" -gt 600 ]; then
        echo
        echo "WARNING: the newest DS log ($ds_name) started $((gap / 60)) min away from the newest"
        echo "robot log ($(basename "$newest_named")). If the robot was turned on again after the match,"
        echo "grab more: tools/collect-ds-logs.sh --last 3"
      fi
    fi
  fi
fi
df -h "$USB" 2>/dev/null | tail -1 | awk '{print "Space left on stick: " $4}' || true
echo
echo "Done. Eject the stick safely, then on the analysis computer run:"
echo "  python .claude/skills/analyze-wpilog/scripts/analyze.py ingest --report"
