#!/usr/bin/env python3
"""Robot log analyzer for the 2026 Offseason CloneBot (AdvantageKit .wpilog files).

Commands (run with --help on any of them for options):
  find                       list .wpilog files, newest first
  info LOG                   match info, enabled time, and the keys in the log
  report LOG                 full health report with prioritized findings (markdown)
  diagnose LOG [MECH]        evidence + ranked mechanical/electrical/programming causes
  timeline LOG               mode changes, state transitions, commands, alerts, faults
  series LOG KEY [KEY ...]   print values of keys over time (sample-and-hold)
  compare SIMLOG             replay log: findings fixed/added by a code change
  ingest [USB] [--report]    copy logs off the robot USB stick, pair DS logs, write reports
  ds DSLOG                   summarize a Driver Station log on its own

Times are seconds since the log started. Pure Python 3.8+, no packages needed.
"""

from __future__ import annotations

import argparse
import math
import os
import platform
import shutil
import statistics
import string
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import robot_profile as P  # noqa: E402
from dslog import read_dslog, read_dsevents  # noqa: E402
from wpilog import Series, WpiLog, pose2d_tuple  # noqa: E402

DT = 0.02  # analysis grid, matches the 20 ms robot loop
REPO_ROOT = Path(__file__).resolve().parents[4]

SEVERITY_ORDER = {"CRITICAL": 0, "WARN": 1, "INFO": 2}
IDLE_STATE_NAMES = {"REST", "IDLE", "STOWED"}


# ============================================================================ helpers


@dataclass
class Finding:
    severity: str  # CRITICAL, WARN, INFO
    category: str  # Electrical, Mechanical, Programming, Tuning, Config
    area: str
    message: str
    time: float | None = None


class Log:
    """WpiLog plus the robot-mode timeline and key fallbacks used by every analysis."""

    def __init__(self, path: str, outputs: str = "auto", ds_path: str | None = None):
        """ds_path: a .dslog to pair with, None to find one automatically, or "none" to skip."""
        self.path = path
        self.raw = WpiLog.open(path)
        self._cache: dict[str, Series | None] = {}
        # A replay (_sim) log holds the original RealOutputs AND the recomputed ReplayOutputs.
        # "auto" analyzes the recomputed decisions when they exist, since that is what the
        # current code did with the recorded inputs.
        has_replay = any(k.startswith("ReplayOutputs/") for k in self.raw.entries)
        if outputs == "auto":
            outputs = "replay" if has_replay else "real"
        self.outputs = outputs
        self.out_prefix = "ReplayOutputs/" if outputs == "replay" else "RealOutputs/"
        self.other_prefix = "RealOutputs/" if outputs == "replay" else "ReplayOutputs/"
        self.end = self.raw.duration
        enabled = self.get(*P.DS_ENABLED)
        if enabled is None or not len(enabled):
            # No Driver Station data (unusual): analyze the whole log as if enabled.
            self.enabled_intervals = [(0.0, self.end)]
            self.has_ds = False
        else:
            self.enabled_intervals = intervals_where(enabled, bool, self.end)
            self.has_ds = True
        self.auto = self.get(*P.DS_AUTO)
        self.test = self.get(*P.DS_TEST)
        self.grid = grid_for(self.enabled_intervals)
        self.ds = self.ds_events = None
        self.ds_note = ""
        if ds_path != "none":
            self._attach_ds(ds_path)

    def _attach_ds(self, ds_path: str | None):
        """Finds the Driver Station log for this match (same folder or a dslogs/ folder next to it)
        and lines its clock up with this log's."""
        self.epoch0 = akit_epoch_offset(self.raw)
        here = Path(self.path).resolve().parent
        if ds_path:
            candidates = [Path(ds_path)]
        else:
            candidates = [p for d in (here, here / "dslogs", here.parent / "dslogs") if d.is_dir()
                          for p in d.glob("*.dslog")]
        if not candidates:
            return
        if self.epoch0 is None:
            self.ds_note = "DS logs found, but this log has no valid clock to line them up with."
            return
        match = best_ds_match(self.epoch0, self.epoch0 + self.end, candidates)
        if match is None:
            self.ds_note = "No DS log overlaps this log's time range."
            return
        self.ds = read_dslog(match)
        events = match.with_suffix(".dsevents")
        if events.is_file():
            try:
                self.ds_events = read_dsevents(events)
            except ValueError:
                pass
        self.ds_note = f"Driver Station log: {match.name}"

    def ds_to_log_time(self, ds_t: float, start_unix: float | None = None) -> float:
        """Converts a time in a DS file (seconds since it started) to this log's time."""
        base = self.ds.start_unix if start_unix is None else start_unix
        return base + ds_t - self.epoch0

    def resolve(self, key: str) -> str | None:
        """Profile keys say RealOutputs/; map them to the chosen outputs tree."""
        if key.startswith("RealOutputs/"):
            chosen = self.out_prefix + key[len("RealOutputs/"):]
            return chosen if self.raw.has(chosen) else None
        return key if self.raw.has(key) else None

    def keys(self) -> list[str]:
        """Every key except the outputs tree that is not being analyzed."""
        return [k for k in self.raw.keys() if not k.startswith(self.other_prefix)]

    def get(self, *keys: str) -> Series | None:
        """First of keys present in the log, decoded (cached)."""
        for key in keys:
            if key is None:
                continue
            if key in self._cache:
                if self._cache[key] is not None:
                    return self._cache[key]
                continue
            resolved = self.resolve(key)
            series = self.raw.series(resolved) if resolved else None
            self._cache[key] = series
            if series is not None:
                return series
        return None

    def mode_at(self, t: float) -> str:
        if not self.has_ds:
            return "?"
        if not any(a <= t <= b for a, b in self.enabled_intervals):
            return "disabled"
        if self.auto is not None and self.auto.at(t):
            return "auto"
        if self.test is not None and self.test.at(t):
            return "test"
        return "teleop"

    def stamp(self, t: float | None) -> str:
        if t is None:
            return ""
        return f"t={t:.2f}s [{self.mode_at(t)}]"

    def enabled_time(self) -> float:
        return sum(b - a for a, b in self.enabled_intervals)


def intervals_where(series: Series, predicate, end: float) -> list[tuple[float, float]]:
    """Sample-and-hold intervals where predicate(value) is true."""
    out, start = [], None
    for t, v in zip(series.times, series.values):
        on = bool(predicate(v))
        if on and start is None:
            start = t
        elif not on and start is not None:
            out.append((start, t))
            start = None
    if start is not None:
        out.append((start, end))
    return [(a, b) for a, b in out if b > a]


def akit_epoch_offset(raw: WpiLog) -> float | None:
    """Unix time of the log's t=0, from SystemStats/EpochTimeMicros (valid once the DS sets the
    roboRIO clock). None if the log never had a valid clock."""
    s = raw.series("SystemStats/EpochTimeMicros")
    if s is None:
        return None
    offsets = [v / 1e6 - t for t, v in zip(s.times, s.values) if isinstance(v, int) and v > 1.5e15]
    return statistics.median(offsets) if offsets else None


def best_ds_match(akit_start: float, akit_end: float, candidates: list[Path]) -> Path | None:
    """The .dslog overlapping [akit_start, akit_end] (unix seconds) the most, or None."""
    best, best_overlap = None, 0.0
    for p in candidates:
        try:
            ds = read_dslog(p)
        except (ValueError, OSError):
            continue
        overlap = min(akit_end, ds.end_unix) - max(akit_start, ds.start_unix)
        if overlap > best_overlap:
            best, best_overlap = p, overlap
    return best


def grid_for(intervals: list) -> list[float]:
    pts = []
    for a, b in intervals:
        n = int((b - a) / DT)
        pts.extend(a + k * DT for k in range(n + 1))
    return pts


def sample(series: Series | None, grid: list[float], index: int | None = None) -> list:
    """Sample-and-hold values of series at each grid time (grid must be sorted)."""
    if series is None:
        return [None] * len(grid)
    out, i, n = [], -1, len(series.times)
    times, values = series.times, series.values
    for t in grid:
        while i + 1 < n and times[i + 1] <= t:
            i += 1
        v = values[i] if i >= 0 else None
        if index is not None and v is not None:
            v = v[index] if isinstance(v, (list, tuple)) and index < len(v) else None
        out.append(v)
    return out


def num(v):
    return v if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) else None


def fmt(x, digits=1):
    return "n/a" if x is None else f"{x:.{digits}f}"


def runs(grid: list[float], flags: list[bool], min_len: float = 0.0) -> list[tuple[float, float]]:
    """Contiguous runs of True on the grid, as (start, end) times, at least min_len long."""
    out, start, prev = [], None, None
    for t, f in zip(grid, flags):
        if f and start is None:
            start = t
        elif f and prev is not None and t - prev > DT * 1.5:  # gap between enabled periods
            out.append((start, prev))
            start = t
        elif not f and start is not None:
            out.append((start, prev))
            start = None
        prev = t
    if start is not None:
        out.append((start, prev))
    return [(a, b) for a, b in out if b - a >= min_len]


def total(rs) -> float:
    return sum(b - a for a, b in rs)


def window_grid(log: Log, start: float | None, end: float | None) -> list[float]:
    lo = 0.0 if start is None else start
    hi = log.end if end is None else end
    return [t for t in log.grid if lo <= t <= hi]


# ============================================================================ analyses


def analyze_mechanism(log: Log, name: str, grid: list[float]) -> tuple[dict, list[Finding]]:
    m = P.MECHANISMS[name]
    label = m["label"]
    f: list[Finding] = []
    ev: dict = {"name": name, "label": label, "present": False}
    if not grid:
        return ev, f

    multi = "motor_names" in m
    meas_s = log.get(m["measured"])
    sp_s = log.get(m["setpoint"])
    if meas_s is None and sp_s is None:
        return ev, f
    ev["present"] = True

    meas = [num(v) for v in sample(meas_s, grid)]
    sp = [num(v) for v in sample(sp_s, grid)]
    vel_s = log.get(m.get("velocity")) if m.get("velocity") else None
    vel = [num(v) for v in sample(vel_s, grid)] if vel_s else [None] * len(grid)
    applied = sample(log.get(m["applied_state"]), grid)
    desired = sample(log.get(m["desired_state"]), grid)
    current = sample(log.get(m["current_state"]), grid)
    lead = 0 if multi else None
    volts = [num(v) for v in sample(log.get(m["volts"]), grid, lead)]
    stator = [num(v) for v in sample(log.get(m["stator"]), grid, lead)]
    supply = [num(v) for v in sample(log.get(m["supply"]), grid, lead)]
    battery = [num(v) for v in sample(log.get(*P.BATTERY), grid)]
    tol = m["tolerance"]
    kind = m["kind"]

    # --- what the code asked for -------------------------------------------------
    sp_by_state: dict[str, set] = {}
    for a, s in zip(applied, sp):
        if a is not None and s is not None:
            sp_by_state.setdefault(str(a), set()).add(round(s, 2))
    ev["setpoints_by_state"] = {k: sorted(v) for k, v in sp_by_state.items()}
    ev["desired_states_seen"] = sorted({str(d) for d in desired if d is not None})
    for state, values in sp_by_state.items():
        if state not in IDLE_STATE_NAMES and values == {0.0} and kind != "position":
            f.append(Finding("WARN", "Programming", label,
                             f"state {state} was applied but its setpoint is 0 {m['units']} "
                             f"(placeholder value in the state enum?)."))
    if kind == "position" and len(sp_by_state) > 1 and all(v == {0.0} for v in sp_by_state.values()):
        f.append(Finding("WARN", "Programming", label,
                         "every applied state has setpoint 0 deg, so the mechanism never moves "
                         "(state enum setpoints are still placeholders)."))

    # --- tracking error ----------------------------------------------------------
    if kind == "velocity":
        active = [s is not None and x is not None and abs(s) > tol for s, x in zip(sp, meas)]
    else:
        active = [s is not None and x is not None for s, x in zip(sp, meas)]
    errs = [(t, x - s) for t, s, x, a in zip(grid, sp, meas, active) if a]
    ev["active_seconds"] = len(errs) * DT
    if errs:
        abs_e = sorted(abs(e) for _, e in errs)
        worst_t, worst_e = max(errs, key=lambda te: abs(te[1]))
        ev["mean_abs_error"] = statistics.fmean(abs_e)
        ev["p95_abs_error"] = abs_e[int(0.95 * (len(abs_e) - 1))]
        ev["max_abs_error"] = abs(worst_e)
        ev["max_error_time"] = worst_t
        ev["out_of_tol_fraction"] = sum(e > tol for e in abs_e) / len(abs_e)

    # --- settling after each setpoint change ------------------------------------
    steps, settles, unsettled = [], [], []
    last_sp = None
    for i, (t, s) in enumerate(zip(grid, sp)):
        if s is None:
            continue
        if last_sp is not None and abs(s - last_sp) > tol and (kind != "velocity" or abs(s) > tol):
            steps.append((i, t, last_sp, s))
        last_sp = s
    for k, (i, t, old, new) in enumerate(steps):
        stop = steps[k + 1][0] if k + 1 < len(steps) else len(grid)
        settled_at, overshoot = None, 0.0
        direction = 1 if new > old else -1
        for j in range(i, stop):
            if grid[j] - grid[max(j - 1, 0)] > DT * 1.5 and j > i:
                break  # robot disabled mid-move
            x = meas[j]
            if x is None:
                continue
            overshoot = max(overshoot, (x - new) * direction)
            if abs(x - new) <= tol:
                hold_end = j + int(0.1 / DT)
                if all(meas[q] is not None and abs(meas[q] - new) <= tol
                       for q in range(j, min(hold_end, stop))):
                    settled_at = grid[j]
                    break
        if settled_at is None:
            end_x = next((meas[q] for q in range(stop - 1, i - 1, -1) if meas[q] is not None), None)
            unsettled.append((t, old, new, end_x, grid[stop - 1] - t))
        else:
            settles.append((settled_at - t, t, old, new, overshoot))
    ev["setpoint_changes"] = len(steps)
    ev["settle_times"] = settles
    ev["unsettled"] = unsettled
    if settles:
        ev["median_settle"] = statistics.median(s[0] for s in settles)
        ev["worst_settle"] = max(settles, key=lambda s: s[0])
        if kind == "position":
            ev["max_overshoot"] = max(s[4] for s in settles)
    real_unsettled = [u for u in unsettled if u[4] > 0.5]
    for t, old, new, end_x, dur in real_unsettled[:3]:
        f.append(Finding("WARN", "Unknown", label,
                         f"never reached {new:.1f} {m['units']} (from {old:.1f}); held "
                         f"{dur:.1f}s and ended at {fmt(end_x)} {m['units']}. Run diagnose.", t))
    if settles and kind == "position" and ev.get("max_overshoot", 0) > 3 * tol:
        f.append(Finding("INFO", "Tuning", label,
                         f"overshoot up to {ev['max_overshoot']:.1f} {m['units']} "
                         f"(tolerance {tol}); lower kP or Motion Magic accel, or add kD."))

    # --- oscillation while the setpoint is constant ------------------------------
    crossings, prev_sign, osc_times = 0, 0, []
    for t, s, x in zip(grid, sp, meas):
        if s is None or x is None:
            continue
        e = x - s
        if abs(e) > tol:
            sign = 1 if e > 0 else -1
            if prev_sign and sign != prev_sign:
                crossings += 1
                osc_times.append(t)
            prev_sign = sign
    ev["error_sign_flips"] = crossings
    dense = [t for k, t in enumerate(osc_times) if k >= 4 and t - osc_times[k - 4] < 2.0]
    if dense:
        f.append(Finding("WARN", "Tuning", label,
                         f"oscillating around the setpoint ({crossings} overshoot crossings); "
                         "kP too high or needs kD.", dense[0]))

    # --- direction / inversion ---------------------------------------------------
    if kind in ("velocity", "position"):
        motion = vel if kind == "position" else meas
        wrong = []
        for t, s, x, v, mv in zip(grid, sp, meas, volts, motion):
            if kind == "velocity":
                bad = s is not None and x is not None and abs(s) > 3 * tol and x * s < 0 and abs(x) > tol
            else:
                bad = (v is not None and mv is not None and abs(v) > 2.0 and abs(mv) > 5.0 and v * mv < 0)
            wrong.append(bad)
        wr = runs(grid, wrong, 0.2)
        ev["wrong_direction_seconds"] = total(wr)
        if wr:
            f.append(Finding("CRITICAL", "Programming", label,
                             "moved opposite to the commanded direction for "
                             f"{total(wr):.1f}s. Check motor Inverted config / follower "
                             "alignment / sensor direction.", wr[0][0]))

    # --- stall: pushing hard but not moving --------------------------------------
    motion = vel if kind == "position" else meas
    limit = m["stator_limit"]
    stall_flags = []
    for v, c, mv in zip(volts, stator, motion):
        still = mv is not None and abs(mv) < (3.0 if kind == "position" else 50.0)
        stall_flags.append(v is not None and c is not None and abs(v) > 3.0 and abs(c) > 0.4 * limit and still)
    stalls = runs(grid, stall_flags, 0.3)
    ev["stalls"] = stalls
    if stalls:
        f.append(Finding("WARN", "Mechanical", label,
                         f"stalled {len(stalls)}x ({total(stalls):.1f}s total): high voltage and "
                         "current but not moving. Jam, binding, hard stop, or game piece stuck.",
                         stalls[0][0]))

    # --- current limits, saturation ----------------------------------------------
    at_stator = runs(grid, [c is not None and abs(c) >= 0.95 * limit for c in stator], 0.1)
    at_supply = runs(grid, [c is not None and abs(c) >= 0.95 * m["supply_limit"] for c in supply], 0.1)
    sat = runs(grid, [v is not None and b is not None and abs(v) >= min(11.0, b - 0.5)
                      for v, b in zip(volts, battery)], 0.5)
    ev["peak_stator"] = max((abs(c) for c in stator if c is not None), default=None)
    ev["peak_supply"] = max((abs(c) for c in supply if c is not None), default=None)
    ev["stator_limit_seconds"] = total(at_stator)
    ev["supply_limit_seconds"] = total(at_supply)
    ev["saturated_seconds"] = total(sat)
    if total(at_stator) > 1.0:
        f.append(Finding("WARN", "Mechanical", label,
                         f"at its {limit:.0f} A stator limit for {total(at_stator):.1f}s; the "
                         "mechanism is loaded harder than expected (or limit too low).",
                         at_stator[0][0]))
    if total(sat) > 1.0 and kind != "voltage":
        f.append(Finding("INFO", "Tuning", label,
                         f"output saturated (~full battery voltage) for {total(sat):.1f}s; "
                         "setpoint may be unreachable or the profile too aggressive.", sat[0][0]))

    # --- commanded but no output ---------------------------------------------------
    no_out = []
    for s, x, v in zip(sp, meas, volts):
        need = s is not None and x is not None and abs(x - s) > 3 * tol
        no_out.append(need and v is not None and abs(v) < 0.25)
    dead = runs(grid, no_out, 0.5)
    ev["no_output_seconds"] = total(dead)
    if dead:
        lim = m.get("soft_limits")
        hint = ""
        if lim is not None:
            hint = f" Soft limits are {lim[0]:.0f}-{lim[1]:.0f} {m['units']}; a target past them is blocked."
        f.append(Finding("WARN", "Programming", label,
                         f"far from setpoint but output ~0 V for {total(dead):.1f}s "
                         f"(soft limit, neutral/disabled request, or wrong control request).{hint}",
                         dead[0][0]))

    # --- soft limits -----------------------------------------------------------------
    lim = m.get("soft_limits")
    if lim is not None:
        lo, hi = lim
        bad_sp = [(t, s) for t, s in zip(grid, sp) if s is not None and (s < lo - 0.01 or s > hi + 0.01)]
        if bad_sp:
            f.append(Finding("WARN", "Programming", label,
                             f"setpoint {bad_sp[0][1]:.1f} {m['units']} is outside the soft limits "
                             f"{lo:.0f}-{hi:.0f}; it can never be reached.", bad_sp[0][0]))
        past = [(t, x) for t, x in zip(grid, meas) if x is not None and (x < lo - 2 or x > hi + 2)]
        if past:
            f.append(Finding("WARN", "Mechanical", label,
                             f"measured {past[0][1]:.1f} {m['units']}, beyond the soft limits "
                             f"{lo:.0f}-{hi:.0f}: encoder zero is wrong (not homed/slipped) or "
                             "the mechanism was back-driven.", past[0][0]))

    # --- desired vs confirmed state ----------------------------------------------
    mism = runs(grid, [d is not None and c is not None and d != c for d, c in zip(desired, current)], 0.0)
    ev["state_mismatch_seconds"] = total(mism)
    long = [r for r in mism if r[1] - r[0] > 2.0]
    if long:
        a, b = max(long, key=lambda r: r[1] - r[0])
        idx = grid.index(a) if a in grid else 0
        f.append(Finding("WARN", "Unknown", label,
                         f"desired state {desired[idx]} not confirmed for {b - a:.1f}s "
                         f"(current stayed {current[idx]}). Run diagnose.", a))
    gated = runs(grid, [d is not None and a is not None and str(d) != str(a)
                        for d, a in zip(desired, applied)], 0.5)
    ev["state_machine_override_seconds"] = total(gated)

    # --- temperature, connectivity ------------------------------------------------
    temps = sample(log.get(m["temp"]), grid)
    flat_temps = [x for v in temps if v is not None for x in (v if isinstance(v, list) else [v])]
    ev["max_temp"] = max(flat_temps, default=None)
    conn_s = log.get(m["connected"])
    ev["disconnects"] = []
    if conn_s is not None:
        names = m.get("motor_names", [None])
        for k, motor in enumerate(names):
            idx = k if multi else None
            vals = sample(conn_s, grid, idx)
            drops = runs(grid, [v is False for v in vals], 0.0)
            drops = [d for d in drops if d[0] > 3.0]  # ignore boot-up
            if drops:
                who = f"{label} {motor}" if motor else label
                ev["disconnects"].append((who, drops))
                f.append(Finding("CRITICAL", "Electrical", who,
                                 f"motor disconnected {len(drops)}x ({total(drops):.1f}s). Check "
                                 "CAN wiring/connectors and power to that controller.", drops[0][0]))

    # --- multi-motor drum: share of the load --------------------------------------
    if multi:
        names = m["motor_names"]
        st_all = log.get(m["stator"])
        means = []
        for k in range(len(names)):
            vals = [num(v) for v, a in zip(sample(st_all, grid, k), active) if a]
            vals = [abs(v) for v in vals if v is not None]
            means.append(statistics.fmean(vals) if vals else None)
        ev["per_motor_mean_stator"] = dict(zip(names, means))
        good = [x for x in means if x is not None]
        if len(good) == len(names) and max(good) > 10 and max(good) > 1.6 * max(min(good), 1.0):
            hi_k = means.index(max(good))
            lo_k = means.index(min(good))
            f.append(Finding("WARN", "Mechanical", label,
                             f"motors share load unevenly: {names[hi_k]} averages {max(good):.0f} A "
                             f"vs {names[lo_k]} {min(good):.0f} A. A motor fighting the others "
                             "(follower direction) or a binding/loose belt on one stage."))
    return ev, f


def analyze_states(log: Log) -> tuple[list, list[Finding]]:
    """Every StateHandler Desired*/Current* pair, discovered from the log."""
    rows, f = [], []
    prefix_keys = [k for k in log.keys() if "/StateHandler/" in "/" + k]
    for k in prefix_keys:
        leaf = k.rsplit("/", 1)[-1]
        if not leaf.startswith("Desired"):
            continue
        cur_key = k[: -len(leaf)] + "Current" + leaf[len("Desired"):]
        des, cur = log.raw.series(k), log.raw.series(cur_key)
        if des is None or cur is None or des.type != "string":
            continue  # only enum state pairs (e.g. DesiredHeading has no Current partner)
        g = log.grid
        d = sample(des, g)
        c = sample(cur, g) if cur is not None else [None] * len(g)
        changes = sum(1 for a, b in zip(des.values, des.values[1:]) if a != b)
        mism = runs(g, [x is not None and y is not None and x != y for x, y in zip(d, c)])
        longest = max(mism, key=lambda r: r[1] - r[0], default=None)
        rows.append((k.split("StateHandler/")[-1], changes, total(mism), longest))
        # A desired state flipping back and forth every loop means two writers are fighting.
        flips = [t for t, a, b in zip(des.times[1:], des.values, des.values[1:]) if a != b]
        fast = [t for i, t in enumerate(flips) if i >= 10 and t - flips[i - 10] < 1.0]
        if fast:
            f.append(Finding("WARN", "Programming", "StateHandler",
                             f"{k.split('StateHandler/')[-1]} changed 10+ times within 1s: "
                             "two commands are probably writing it at once.", fast[0]))
    return rows, f


def analyze_shooting(log: Log) -> tuple[dict, list[Finding]]:
    ev, f = {}, []
    g = log.grid
    shoot = [False] * len(g)
    for key in P.SHOOTING["shoot_commands"]:
        s = log.get(key)
        if s is not None:
            shoot = [a or bool(b) for a, b in zip(shoot, sample(s, g))]
    attempts = runs(g, shoot)
    ev["attempts"] = len(attempts)
    ev["attempt_seconds"] = total(attempts)
    if not attempts:
        return ev, f
    ready = sample(log.get(P.SHOOTING["ready"]), g)
    hood, drum = P.MECHANISMS["hood"], P.MECHANISMS["drum"]
    hood_err = [None if (a is None or b is None) else abs(a - b) for a, b in
                zip(sample(log.get(hood["measured"]), g), sample(log.get(hood["setpoint"]), g))]
    drum_sp = sample(log.get(drum["setpoint"]), g)
    drum_err = [None if (a is None or b is None) else abs(a - b) for a, b in
                zip(sample(log.get(drum["measured"]), g), drum_sp)]
    head_err = sample(log.get(P.SWERVE["heading_error"]), g)
    swerve_state = sample(log.get("RealOutputs/StateHandler/Swerve/State"), g)
    blockers = {"drum not at speed": 0, "hood not at angle": 0, "drum setpoint is 0": 0,
                "heading not on target": 0}
    ready_n = shoot_n = 0
    to_ready = []
    for a, b in attempts:
        first = next((t for t, r in zip(g, ready) if a <= t <= b and r), None)
        if first is not None:
            to_ready.append(first - a)
    for i, s in enumerate(shoot):
        if not s:
            continue
        shoot_n += 1
        if ready[i]:
            ready_n += 1
            continue
        if num(drum_sp[i]) is not None and abs(drum_sp[i]) < 1:
            blockers["drum setpoint is 0"] += 1
        if drum_err[i] is not None and drum_err[i] >= drum["tolerance"]:
            blockers["drum not at speed"] += 1
        if hood_err[i] is not None and hood_err[i] >= hood["tolerance"]:
            blockers["hood not at angle"] += 1
        if (swerve_state[i] == "FACING_HEADING" and num(head_err[i]) is not None
                and abs(head_err[i]) >= P.SWERVE["heading_tolerance"]):
            blockers["heading not on target"] += 1
    ev["ready_fraction"] = ready_n / shoot_n if shoot_n else None
    ev["blockers_seconds"] = {k: v * DT for k, v in blockers.items() if v}
    ev["median_time_to_ready"] = statistics.median(to_ready) if to_ready else None
    ev["never_ready_attempts"] = len(attempts) - len(to_ready)
    if ev["never_ready_attempts"]:
        top = max(blockers, key=blockers.get)
        cat = "Programming" if top == "drum setpoint is 0" else "Unknown"
        f.append(Finding("WARN", cat, "Shooting",
                         f"{ev['never_ready_attempts']} of {len(attempts)} shoot attempts never "
                         f"became ready, so nothing was fed. Main blocker: {top}."))
    homed = log.get(P.SHOOTING["homed"])
    if homed is not None:
        ev["homing_results"] = list(zip(homed.times, homed.values))
        for t, v in ev["homing_results"]:
            if not v:
                f.append(Finding("WARN", "Mechanical", "Shooter hood",
                                 "hood homing did not find the hard stop (timed out or cancelled); "
                                 "hood angles are unreliable until it homes.", t))
    else:
        ev["homing_results"] = []
    return ev, f


def analyze_electrical(log: Log) -> tuple[dict, list[Finding]]:
    ev, f = {}, []
    g = log.grid
    bat = [num(v) for v in sample(log.get(*P.BATTERY), g)]
    vals = [v for v in bat if v is not None]
    if vals:
        ev["battery_min"] = min(vals)
        ev["battery_min_time"] = g[bat.index(min(vals))]
        ev["battery_mean"] = statistics.fmean(vals)
        low = runs(g, [v is not None and v < P.BATTERY_WARN_V for v in bat])
        ev["battery_low_seconds"] = total(low)
        if min(vals) < P.BATTERY_CRIT_V:
            f.append(Finding("CRITICAL", "Electrical", "Battery",
                             f"dipped to {min(vals):.2f} V (brownout starts ~6.8 V). Weak/old battery, "
                             "loose main connection, or too much current draw at once.",
                             ev["battery_min_time"]))
        elif total(low) > 2:
            f.append(Finding("WARN", "Electrical", "Battery",
                             f"below {P.BATTERY_WARN_V} V for {total(low):.1f}s; check battery "
                             "charge/health and main breaker connections.", low[0][0]))
    bo = log.get(P.BROWNOUT)
    if bo is not None:
        events = intervals_where(bo, bool, log.end)
        ev["brownouts"] = events
        if events:
            f.append(Finding("CRITICAL", "Electrical", "roboRIO",
                             f"browned out {len(events)}x: motor outputs were cut.", events[0][0]))
    cd = log.get(P.COMMS_DISABLE)
    if cd is not None and cd.values:
        delta = cd.values[-1] - cd.values[0]
        ev["comms_disables"] = delta
        if delta > 0:
            t = next(t for t, v in zip(cd.times, cd.values) if v > cd.values[0])
            f.append(Finding("CRITICAL", "Electrical", "Radio/DS link",
                             f"robot lost DS communication {delta}x (comms disable). Check radio "
                             "power/ethernet and roboRIO-radio cable.", t))
    util = log.get(P.CAN_UTIL)
    if util is not None:
        u = [num(v) for v in sample(util, g)]
        u = [v for v in u if v is not None]
        if u:
            ev["can_util_max"] = max(u) * (100 if max(u) <= 1.0 else 1)
            ev["can_util_mean"] = statistics.fmean(u) * (100 if max(u) <= 1.0 else 1)
            if ev["can_util_mean"] > 70:
                f.append(Finding("WARN", "Electrical", "CAN bus",
                                 f"average utilization {ev['can_util_mean']:.0f}%: lower status "
                                 "signal rates or move devices to a CANivore."))
    for key, label in [(P.CAN_OFF, "bus-off events"), (P.CAN_TX_FULL, "TX buffer full")]:
        s = log.get(key)
        if s is not None and s.values:
            d = s.values[-1] - s.values[0]
            ev[label] = d
            if d > 0:
                t = next(t for t, v in zip(s.times, s.values) if v > s.values[0])
                f.append(Finding("WARN" if label == "TX buffer full" else "CRITICAL", "Electrical",
                                 "CAN bus", f"{d} {label}: CAN wiring fault, missing terminator, "
                                 "or a device shorting the bus.", t))
    loop = log.get(P.LOOP_TIME)
    if loop is not None:
        lv = [(t, v) for t, v in zip(loop.times, loop.values) if num(v) is not None]
        over = [(t, v) for t, v in lv if v > P.LOOP_OVERRUN_MS]
        ev["loop_max_ms"] = max((v for _, v in lv), default=None)
        ev["loop_overruns"] = len(over)
        if len(over) > 50:
            f.append(Finding("WARN", "Programming", "Robot loop",
                             f"{len(over)} loops over {P.LOOP_OVERRUN_MS:.0f} ms (max "
                             f"{ev['loop_max_ms']:.0f} ms): slow code, blocking calls, or GC.",
                             over[0][0]))
    return ev, f


# Routine DS event messages that carry no diagnostic value.
_DS_EVENT_NOISE = ("Info roboRIO: Disk Free", "Info Joystick", "-- FRC Driver Station", "FMS Connected:")


def ds_event_kind(text: str) -> str | None:
    """Classifies a DS event message; None for routine noise."""
    low = text.lower()
    if any(text.startswith(n) for n in _DS_EVENT_NOISE):
        return None
    if "lost communication" in low or "no robot communication" in low:
        return "comms"
    if "brownout" in low or "voltage" in low:
        return "power"
    if "not keeping up" in low or "ds laptop" in low:
        return "laptop"
    if "joystick" in low and ("unplugged" in low or "disconnected" in low or "lost" in low):
        return "joystick"
    if "watchdog" in low or "loop time" in low or "overrun" in low:
        return "code"
    if "no robot code" in low or "code start" in low or "robot code" in low:
        return "code"
    if "error" in low or "exception" in low or "warning" in low:
        return "error"
    return "info"


def analyze_ds(log: Log) -> tuple[dict, list[Finding]]:
    """Driver Station side of the match: radio link, DS-reported power, PD channel currents."""
    ev, f = {"note": log.ds_note}, []
    ds = log.ds
    if ds is None:
        return ev, f
    lo, hi = 0.0, log.end
    idx = [i for i, t in enumerate(ds.times) if lo <= log.ds_to_log_time(t) <= hi]
    en = [i for i in idx if ds.enabled(i)]
    ev["enabled_seconds"] = len(en) * DT
    if en:
        trips = sorted(ds.trip_ms[i] for i in en)
        losses = [ds.packet_loss[i] for i in en]
        ev["trip_median_ms"] = statistics.median(trips)
        ev["trip_p95_ms"] = trips[int(0.95 * (len(trips) - 1))]
        ev["loss_mean"] = statistics.fmean(losses)
        ev["loss_max"] = max(losses)
        ev["wifi_db_mean"] = statistics.fmean(ds.wifi_db[i] for i in en)
        ev["bandwidth_max_mb"] = max(ds.wifi_mb[i] for i in en)
        ev["ds_battery_min"] = min(ds.battery[i] for i in en)
        times = [log.ds_to_log_time(ds.times[i]) for i in en]
        lossy = runs(times, [ds.packet_loss[i] >= 0.2 for i in en], 1.0)
        slow = runs(times, [ds.trip_ms[i] >= 50 for i in en], 1.0)
        if lossy:
            f.append(Finding("WARN", "Electrical", "Radio link",
                             f"packet loss at or above 20% for {total(lossy):.1f}s: radio placement/power, "
                             "ethernet to the radio, or field interference.", lossy[0][0]))
        if slow:
            f.append(Finding("WARN", "Electrical", "Radio link",
                             f"round-trip time at or above 50 ms for {total(slow):.1f}s (driving feels laggy).",
                             slow[0][0]))
        dog = runs(times, [ds.watchdog[i] for i in en], 0.1)
        if dog:
            f.append(Finding("WARN", "Programming", "Robot code",
                             f"motor safety watchdog tripped {len(dog)}x ({total(dog):.1f}s) while enabled: "
                             "loop overruns or a blocking call.", dog[0][0]))
    # Lost comms: the robot stops answering after it had been talking to the DS.
    first_conn = next((i for i in idx if ds.connected[i]), None)
    if first_conn is not None:
        times = [log.ds_to_log_time(ds.times[i]) for i in idx]
        drops = [r for r in runs(times, [not ds.connected[i] for i in idx], 0.1)
                 if r[0] > log.ds_to_log_time(ds.times[first_conn])]
        ev["comms_drops"] = drops
    # Power distribution channels.
    if ds.pd_type != "none" and en:
        nch = max(len(ds.pd_currents[i]) for i in en)
        peaks, means = [], []
        for ch in range(nch):
            vals = [ds.pd_currents[i][ch] for i in en if ch < len(ds.pd_currents[i])]
            peaks.append(max(vals, default=0.0))
            means.append(statistics.fmean(vals) if vals else 0.0)
        ev["pd_type"] = ds.pd_type
        ev["pd_channels"] = [(ch, P.PD_CHANNELS.get(ch, ""), peaks[ch], means[ch]) for ch in range(nch)]
        totals = [sum(ds.pd_currents[i]) for i in en]
        ev["pd_total_peak"] = max(totals)
        ev["pd_total_mean"] = statistics.fmean(totals)
    # DS event messages, on this log's clock.
    ev["events"] = []
    if log.ds_events is not None:
        prev = None
        for t, text in log.ds_events.events:
            kind = ds_event_kind(text)
            lt = log.ds_to_log_time(t, log.ds_events.start_unix)
            if kind is None or text == prev or not (lo <= lt <= hi):
                continue
            prev = text
            ev["events"].append((lt, kind, text))
        for lt, kind, text in ev["events"]:
            if kind == "comms":
                ev.setdefault("comms_lost_messages", []).append(lt)
            if kind == "joystick":
                f.append(Finding("WARN", "Field/DS", "Driver Station",
                                 f"controller problem: {text[:120]}", lt))
            elif kind == "laptop" and "not keeping up" in text.lower():
                f.append(Finding("INFO", "Field/DS", "Driver Station laptop",
                                 "DS laptop not keeping up (CPU/power plan); can cause lag.", lt))
        # Only report one "laptop" finding.
        seen, kept = False, []
        for x in f:
            if x.area == "Driver Station laptop":
                if seen:
                    continue
                seen = True
            kept.append(x)
        f = kept
    # Comms losses: short ones often only show up as the DS's "lost communication" message, not in
    # the 20 ms cycle data, so count both and merge anything within 3 s of each other.
    losses = sorted([a for a, _ in ev.get("comms_drops", [])] + ev.get("comms_lost_messages", []))
    merged = [t for k, t in enumerate(losses) if k == 0 or t - losses[k - 1] > 3.0]
    ev["comms_losses"] = merged
    if merged:
        f.append(Finding("CRITICAL", "Electrical", "Radio link",
                         f"Driver Station lost communication with the robot {len(merged)}x (at "
                         + ", ".join(f"{t:.1f}s" for t in merged[:6]) + "): radio power/brownout, "
                         "loose ethernet at the radio or roboRIO, or a roboRIO/code restart.", merged[0]))
    return ev, f


def analyze_swerve(log: Log) -> tuple[dict, list[Finding]]:
    s, ev, f = P.SWERVE, {}, []
    g = log.grid
    names = s["module_names"]
    for key_name, what in [("drive_connected", "drive motor"), ("steer_connected", "steer motor"),
                           ("encoder_connected", "CANcoder")]:
        series = log.get(s[key_name])
        if series is None:
            continue
        for k, mod in enumerate(names):
            drops = [d for d in runs(g, [v is False for v in sample(series, g, k)]) if d[0] > 3.0]
            if drops:
                f.append(Finding("CRITICAL", "Electrical", f"Swerve {mod} {what}",
                                 f"disconnected {len(drops)}x ({total(drops):.1f}s).", drops[0][0]))
    gyro = log.get(s["gyro_connected"])
    if gyro is not None:
        drops = [d for d in runs(g, [v is False for v in sample(gyro, g)]) if d[0] > 3.0]
        if drops:
            f.append(Finding("CRITICAL", "Electrical", "Pigeon gyro",
                             f"disconnected {len(drops)}x: field-centric driving and odometry break.",
                             drops[0][0]))
    temps = log.get(s["drive_temp"])
    if temps is not None:
        per = []
        for k, mod in enumerate(names):
            vals = [v for v in sample(temps, g, k) if num(v) is not None]
            per.append(max(vals, default=None))
        ev["drive_max_temp"] = dict(zip(names, per))
    st = log.get(s["drive_stator"])
    if st is not None:
        ev["drive_peak_stator"] = {mod: max((abs(v) for v in sample(st, g, k) if num(v) is not None), default=None)
                                   for k, mod in enumerate(names)}
        means = []
        for k in range(len(names)):
            vals = [abs(v) for v in sample(st, g, k) if num(v) is not None]
            means.append(statistics.fmean(vals) if vals else None)
        ev["drive_mean_stator"] = dict(zip(names, means))
        good = [x for x in means if x is not None]
        if len(good) == 4 and max(good) > 8 and max(good) > 1.5 * min(good):
            k = means.index(max(good))
            f.append(Finding("WARN", "Mechanical", f"Swerve {names[k]} drive",
                             f"averages {max(good):.0f} A vs {min(good):.0f} A on the lightest "
                             "module: dragging bearing/gear, tight belt, or wheel rubbing."))
    daq = log.get(s["failed_daqs"])
    if daq is not None and daq.values:
        d = daq.values[-1] - daq.values[0]
        ev["failed_odometry_daqs"] = d
        if d > 100:
            f.append(Finding("WARN", "Electrical", "Swerve odometry",
                             f"{d} failed odometry reads: CAN bus overloaded or a device dropping."))
    he = log.get(s["heading_error"])
    state = log.get("RealOutputs/StateHandler/Swerve/State")
    if he is not None and state is not None:
        facing = [a == "FACING_HEADING" for a in sample(state, g)]
        errs = [abs(v) for v, fa in zip(sample(he, g), facing) if fa and num(v) is not None]
        if errs:
            ev["heading_mean_abs_error"] = statistics.fmean(errs)
            ev["heading_in_tol_fraction"] = sum(e < s["heading_tolerance"] for e in errs) / len(errs)
    return ev, f


def analyze_vision(log: Log) -> tuple[dict, list[Finding]]:
    ev, f = {}, []
    g = log.grid
    for cam in P.VISION_CAMERAS:
        conn = log.get(f"Vision/{cam}/Connected")
        if conn is None:
            continue
        c = sample(conn, g)
        est = sample(log.get(f"Vision/{cam}/HasEstimate"), g)
        acc = sample(log.get(f"RealOutputs/Vision/{cam}/Accepted"), g)
        dist = [num(v) for v, e in zip(sample(log.get(f"Vision/{cam}/AverageTagDistanceMeters"), g), est) if e]
        n = len(g) or 1
        ev[cam] = {
            "connected": sum(1 for v in c if v) / n,
            "has_estimate": sum(1 for v in est if v) / n,
            "accepted": sum(1 for v in acc if v) / n,
            "mean_tag_distance": statistics.fmean([d for d in dist if d is not None]) if any(dist) else None,
        }
        if log.has_ds and ev[cam]["connected"] < 0.9:
            f.append(Finding("WARN", "Electrical", f"Limelight {cam}",
                             f"connected only {ev[cam]['connected'] * 100:.0f}% of enabled time "
                             "(PoE/ethernet/power)."))
    return ev, f


def analyze_alerts_and_console(log: Log) -> tuple[dict, list[Finding]]:
    ev, f = {"alerts": {}, "console": []}, []
    for key in log.keys():
        leaf = key.rsplit("/", 1)[-1]
        if leaf not in ("errors", "warnings") or log.raw.type_of(key) != "string[]":
            continue
        s = log.raw.series(key)
        first: dict[str, float] = {}
        for t, arr in zip(s.times, s.values):
            for msg in arr or []:
                first.setdefault(msg, t)
        for msg, t in first.items():
            ev["alerts"][msg] = (leaf, t)
            # Disconnect alerts duplicate the per-motor findings; keep the rest.
            if "disconnected" not in msg.lower() and t > 3.0:
                f.append(Finding("WARN" if leaf == "errors" else "INFO", "Unknown", "Alert",
                                 f"{leaf[:-1]}: {msg}", t))
    con = log.get(P.CONSOLE)
    if con is not None:
        seen = set()
        for t, text in zip(con.times, con.values):
            for line in str(text).splitlines():
                low = line.lower()
                if any(w in low for w in ("error", "exception", "warning", "overrun")) and line not in seen:
                    seen.add(line)
                    ev["console"].append((t, line.strip()))
    return ev, f


def analyze_commands(log: Log) -> list:
    rows = []
    for key in log.keys():
        k = key.split("Outputs/", 1)[-1]
        if not k.startswith("Commands/"):
            continue
        s = log.raw.series(key)
        on = intervals_where(s, bool, log.end)
        if on:
            rows.append((k[len("Commands/"):], len(on), total(on), on[0][0]))
    return sorted(rows, key=lambda r: r[3])


def run_all(log: Log):
    findings: list[Finding] = []
    results = {}
    for name in P.MECHANISMS:
        ev, f = analyze_mechanism(log, name, log.grid)
        results[name] = ev
        findings += f
    for key, fn in [("electrical", analyze_electrical), ("shooting", analyze_shooting),
                    ("swerve", analyze_swerve), ("vision", analyze_vision),
                    ("alerts", analyze_alerts_and_console), ("ds", analyze_ds)]:
        ev, f = fn(log)
        results[key] = ev
        findings += f
    rows, f = analyze_states(log)
    results["states"] = rows
    findings += f
    results["commands"] = analyze_commands(log)
    # Temperatures across every motor family.
    for key in log.keys():
        if key.endswith("TemperatureCelsius"):
            s = log.raw.series(key)
            flat = [(t, k, x) for t, v in zip(s.times, s.values)
                    for k, x in enumerate(v if isinstance(v, list) else [v]) if num(x) is not None]
            if flat:
                t, k, hot = max(flat, key=lambda p: p[2])
                if hot >= P.TEMP_WARN_C:
                    sev = "CRITICAL" if hot >= P.TEMP_CRIT_C else "WARN"
                    area = key.replace("TemperatureCelsius", "")
                    if isinstance(s.values[0], list):
                        area += f" motor index {k}"
                    findings.append(Finding(sev, "Mechanical", area,
                                            f"reached {hot:.0f} C. Kraken output drops when hot; "
                                            "check for binding or continuous stall.", t))
    findings.sort(key=lambda x: (SEVERITY_ORDER[x.severity], x.time if x.time is not None else 1e9))
    return results, findings


# ============================================================================ commands


def match_header(log: Log) -> list[str]:
    lines = []
    def last(key):
        s = log.get(key)
        return s.values[-1] if s is not None and s.values else None
    event, mnum, mtype = last("DriverStation/EventName"), last("DriverStation/MatchNumber"), last("DriverStation/MatchType")
    station = last("DriverStation/AllianceStation")
    fms = last("DriverStation/FMSAttached")
    if event or mnum:
        types = {0: "none", 1: "practice", 2: "qualification", 3: "elimination"}
        lines.append(f"- Match: {event or '?'} {types.get(mtype, mtype)} {mnum}, station {station}, FMS {'yes' if fms else 'no'}")
    red = log.get("RealOutputs/StateHandler/IsRedAlliance")
    if red is not None and red.values:
        lines.append(f"- Alliance (as the code saw it): {'red' if red.values[-1] else 'blue'}")
    lines.append(f"- Log length: {log.end:.1f}s, enabled {log.enabled_time():.1f}s "
                 f"in {len(log.enabled_intervals)} period(s)"
                 + ("" if log.has_ds else " (no Driver Station data; whole log analyzed)"))
    auto_t = sum(DT for t in log.grid if log.mode_at(t) == "auto")
    if auto_t:
        lines.append(f"- Auto {auto_t:.1f}s, teleop {log.enabled_time() - auto_t:.1f}s")
    return lines


def cmd_report(args):
    log = Log(args.log, args.outputs, args.ds)
    results, findings = run_all(log)
    out = [f"# Robot log report: {Path(args.log).name}", ""]
    out += match_header(log)
    out += ["", "## Findings", ""]
    if not findings:
        out.append("No problems detected. (Placeholder setpoints and gains can hide problems: "
                   "a mechanism that is never commanded can't fail.)")
    for x in findings:
        out.append(f"- **{x.severity}** [{x.category}] {x.area}: {x.message} {log.stamp(x.time)}".rstrip())

    e = results["electrical"]
    out += ["", "## Electrical", ""]
    out.append(f"- Battery: min {fmt(e.get('battery_min'), 2)} V {log.stamp(e.get('battery_min_time'))}, "
               f"mean {fmt(e.get('battery_mean'), 2)} V, below {P.BATTERY_WARN_V} V for "
               f"{fmt(e.get('battery_low_seconds'))}s")
    out.append(f"- Brownouts: {len(e.get('brownouts', []))}; comms disables: {e.get('comms_disables', 'n/a')}")
    out.append(f"- CAN: mean {fmt(e.get('can_util_mean'))}% / max {fmt(e.get('can_util_max'))}% utilization, "
               f"bus-off {e.get('bus-off events', 'n/a')}, TX full {e.get('TX buffer full', 'n/a')}")
    out.append(f"- Loop: max {fmt(e.get('loop_max_ms'))} ms, {e.get('loop_overruns', 'n/a')} loops over "
               f"{P.LOOP_OVERRUN_MS:.0f} ms")

    out += ["", "## Mechanisms (enabled time only)", "",
            "| Mechanism | Active s | Mean err | p95 err | Max err | Settles (median s) | Never settled | Peak stator A | Max temp C | Stalls |",
            "|---|---|---|---|---|---|---|---|---|---|"]
    for name, m in P.MECHANISMS.items():
        r = results[name]
        if not r.get("present"):
            out.append(f"| {m['label']} | not in log | | | | | | | | |")
            continue
        out.append(
            f"| {m['label']} | {fmt(r.get('active_seconds'))} | {fmt(r.get('mean_abs_error'), 2)} "
            f"| {fmt(r.get('p95_abs_error'), 2)} | {fmt(r.get('max_abs_error'), 2)} "
            f"| {len(r.get('settle_times', []))} ({fmt(r.get('median_settle'), 2)}) "
            f"| {len(r.get('unsettled', []))} | {fmt(r.get('peak_stator'))} | {fmt(r.get('max_temp'))} "
            f"| {len(r.get('stalls', []))} |")
    out.append("")
    out.append("Errors are in each mechanism's units (deg, RPM, or V). Setpoints by applied state:")
    for name, m in P.MECHANISMS.items():
        r = results[name]
        if r.get("setpoints_by_state"):
            pairs = ", ".join(f"{k}={v}" for k, v in r["setpoints_by_state"].items())
            out.append(f"- {m['label']}: {pairs}")
        if r.get("per_motor_mean_stator"):
            pairs = ", ".join(f"{k} {fmt(v)} A" for k, v in r["per_motor_mean_stator"].items())
            out.append(f"  - mean stator per motor while active: {pairs}")

    s = results["shooting"]
    out += ["", "## Shooting", ""]
    if not s.get("attempts"):
        out.append("- No shoot commands ran.")
    else:
        out.append(f"- {s['attempts']} attempts, {s['attempt_seconds']:.1f}s held; ready "
                   f"{fmt((s.get('ready_fraction') or 0) * 100)}% of that time; median time to ready "
                   f"{fmt(s.get('median_time_to_ready'), 2)} s; {s.get('never_ready_attempts')} never ready")
        for k, v in s.get("blockers_seconds", {}).items():
            out.append(f"  - not ready because {k}: {v:.1f}s")
    for t, v in s.get("homing_results", []):
        out.append(f"- Hood homing {'succeeded' if v else 'FAILED'} {log.stamp(t)}")

    out += ["", "## States (desired vs confirmed)", "",
            "| State | Desired changes | Unconfirmed s | Longest unconfirmed |", "|---|---|---|---|"]
    for name, changes, mism, longest in results["states"]:
        lng = f"{longest[1] - longest[0]:.1f}s at {longest[0]:.1f}s" if longest else "-"
        out.append(f"| {name} | {changes} | {mism:.1f} | {lng} |")

    d = results["ds"]
    out += ["", "## Driver Station", ""]
    if log.ds is None:
        out.append("- " + (d.get("note") or "No DS log next to this log (see the match log workflow)."))
    else:
        out.append(f"- {d['note']}; enabled per DS {fmt(d.get('enabled_seconds'))}s")
        if "trip_median_ms" in d:
            out.append(f"- Round trip: median {fmt(d['trip_median_ms'])} ms, 95% {fmt(d['trip_p95_ms'])} ms; "
                       f"packet loss mean {fmt(d['loss_mean'] * 100)}%, max {fmt(d['loss_max'] * 100)}%")
            out.append(f"- Wi-Fi {fmt(d['wifi_db_mean'])} dB mean, bandwidth peak {fmt(d['bandwidth_max_mb'], 2)} Mb/s; "
                       f"DS-reported battery min {fmt(d['ds_battery_min'], 2)} V")
        losses = d.get("comms_losses", [])
        out.append(f"- Comms losses: {len(losses)}" + (" (at " + ", ".join(f"{t:.1f}s" for t in losses[:8]) + ")" if losses else ""))
        if d.get("pd_channels"):
            top = [c for c in sorted(d["pd_channels"], key=lambda c: -c[2]) if c[2] >= 0.5][:8]
            out.append(f"- Power distribution ({d['pd_type']}): total peak {fmt(d['pd_total_peak'])} A, mean "
                       f"{fmt(d['pd_total_mean'])} A while enabled. Highest channels:")
            out.append("")
            out.append("| Channel | Device | Peak A | Mean A |")
            out.append("|---|---|---|---|")
            for ch, name, peak, mean in top:
                out.append(f"| {ch} | {name or '(unmapped)'} | {peak:.1f} | {mean:.1f} |")
            out.append("")
        notable = [e for e in d.get("events", []) if e[1] != "info"]
        if notable:
            out.append("- DS messages:")
            for t, kind, text in notable[:25]:
                out.append(f"  - {t:.1f}s [{kind}] {text[:160]}")

    sw = results["swerve"]
    out += ["", "## Swerve", ""]
    if sw.get("drive_mean_stator"):
        out.append("- Drive mean stator A: " + ", ".join(f"{k} {fmt(v)}" for k, v in sw["drive_mean_stator"].items()))
    if sw.get("drive_max_temp"):
        out.append("- Drive max temp C: " + ", ".join(f"{k} {fmt(v)}" for k, v in sw["drive_max_temp"].items()))
    if "heading_mean_abs_error" in sw:
        out.append(f"- Aiming: mean heading error {sw['heading_mean_abs_error']:.1f} deg, within "
                   f"tolerance {sw['heading_in_tol_fraction'] * 100:.0f}% of the time")
    out.append(f"- Failed odometry reads: {sw.get('failed_odometry_daqs', 'n/a')}")

    v = results["vision"]
    if v:
        out += ["", "## Vision (fraction of enabled time)", "",
                "| Camera | Connected | Has estimate | Accepted | Mean tag dist m |", "|---|---|---|---|---|"]
        for cam, r in v.items():
            out.append(f"| {cam} | {r['connected'] * 100:.0f}% | {r['has_estimate'] * 100:.0f}% "
                       f"| {r['accepted'] * 100:.0f}% | {fmt(r['mean_tag_distance'], 2)} |")

    out += ["", "## Commands", "", "| Command | Times run | Total s | First at |", "|---|---|---|---|"]
    for name, count, secs, first in results["commands"]:
        out.append(f"| {name} | {count} | {secs:.1f} | {first:.1f}s |")

    a = results["alerts"]
    if a["alerts"] or a["console"]:
        out += ["", "## Alerts and console", ""]
        for msg, (kind, t) in a["alerts"].items():
            out.append(f"- {kind[:-1]}: {msg} (first {t:.1f}s)")
        for t, line in a["console"][:25]:
            out.append(f"- console {t:.1f}s: `{line[:160]}`")
    text = "\n".join(out) + "\n"
    if args.out:
        Path(args.out).write_text(text, encoding="utf-8")
        print(f"Report written to {args.out}")
    else:
        print(text)


CAUSE_FILES = {
    "Electrical": "Check: battery, main breaker, PDH slot/breaker, CAN wiring + termination, "
                  "Phoenix Tuner X device status (faults, firmware), motor power leads.",
    "Mechanical": "Check: binding/friction by hand (robot off), belts/chains, hard stops, game "
                  "piece jams, loose set screws/shaft collars, rubbing. Watch it run slowly.",
    "Programming": "Check: the files listed below; enum setpoints, inversions, follower alignment, "
                   "soft limits, control request type, command bindings, state machine guards.",
    "Tuning": "Check: Slot0 gains and Motion Magic in Constants.java; tune kS -> kV/kG -> kP -> kD.",
}


def cmd_diagnose(args):
    log = Log(args.log, args.outputs, args.ds)
    names = [args.mech] if args.mech else list(P.MECHANISMS)
    for n in names:
        if n not in P.MECHANISMS:
            sys.exit(f"Unknown mechanism '{n}'. Choose from: {', '.join(P.MECHANISMS)}")
    grid = window_grid(log, args.start, args.end)
    if not grid:
        sys.exit("No enabled time in that window. (Use timeline to find when the robot was enabled.)")
    win = f"{grid[0]:.1f}s-{grid[-1]:.1f}s" if (args.start is not None or args.end is not None) else "all enabled time"
    e_ev, e_f = analyze_electrical(log)
    e_f += analyze_ds(log)[1]
    e_f = [x for x in e_f if x.time is None or grid[0] <= x.time <= grid[-1]]

    for name in names:
        m = P.MECHANISMS[name]
        ev, f = analyze_mechanism(log, name, grid)
        if not ev.get("present"):
            print(f"## {m['label']}: no data in this log\n")
            continue
        if not args.mech and not f:
            continue  # scanning all mechanisms: only show ones with something wrong
        print(f"## {m['label']} ({m['motors']}) - {win}\n")
        print("Evidence:")
        print(f"- Desired states requested: {', '.join(ev['desired_states_seen']) or 'none'}")
        print(f"- Setpoints by applied state: {ev['setpoints_by_state'] or 'none'}")
        print(f"- Active {fmt(ev.get('active_seconds'))}s; mean |err| {fmt(ev.get('mean_abs_error'), 2)}, "
              f"max {fmt(ev.get('max_abs_error'), 2)} {m['units']} {log.stamp(ev.get('max_error_time'))}; "
              f"tolerance {m['tolerance']}")
        print(f"- Setpoint changes {ev['setpoint_changes']}, settled {len(ev['settle_times'])} "
              f"(median {fmt(ev.get('median_settle'), 2)}s), never settled {len(ev['unsettled'])}")
        for t, old, new, end_x, dur in ev["unsettled"][:5]:
            print(f"    - {old:.1f} -> {new:.1f} at {log.stamp(t)}, ended at {fmt(end_x)} after {dur:.1f}s")
        print(f"- Peak stator {fmt(ev.get('peak_stator'))} A (limit {m['stator_limit']:.0f}, at limit "
              f"{fmt(ev['stator_limit_seconds'])}s); peak supply {fmt(ev.get('peak_supply'))} A "
              f"(limit {m['supply_limit']:.0f}); saturated {fmt(ev['saturated_seconds'])}s")
        print(f"- Stalls: {len(ev['stalls'])}; wrong-direction {fmt(ev.get('wrong_direction_seconds'))}s; "
              f"far from target with ~0 V {fmt(ev['no_output_seconds'])}s; oscillation crossings "
              f"{ev['error_sign_flips']}")
        print(f"- Desired != confirmed {fmt(ev['state_mismatch_seconds'])}s; state machine overrode the "
              f"request {fmt(ev['state_machine_override_seconds'])}s; max temp {fmt(ev.get('max_temp'))} C")
        if ev.get("per_motor_mean_stator"):
            print("- Per-motor mean stator: " + ", ".join(f"{k} {fmt(v)} A" for k, v in ev["per_motor_mean_stator"].items()))
        if ev["disconnects"]:
            for who, drops in ev["disconnects"]:
                print(f"- DISCONNECT {who}: " + ", ".join(f"{a:.1f}-{b:.1f}s" for a, b in drops[:5]))
        print(f"- Battery min in window {fmt(e_ev.get('battery_min'), 2)} V; brownouts "
              f"{len(e_ev.get('brownouts', []))}")

        scores = {"Electrical": 0.0, "Mechanical": 0.0, "Programming": 0.0, "Tuning": 0.0}
        reasons: dict[str, list[str]] = {k: [] for k in scores}
        def add(cat, pts, why):
            scores[cat] += pts
            reasons[cat].append(why)
        # When did this mechanism's problem start? Electrical events only explain it if they line up.
        starts = [r[0] for r in ev["stalls"]] + [u[0] for u in ev["unsettled"]]
        starts += [x.time for x in f if x.time is not None and x.severity != "INFO"]
        problem_t = min(starts) if starts else None

        def near(t):
            return problem_t is None or t is None or abs(t - problem_t) <= 2.0

        if ev["disconnects"]:
            add("Electrical", 5, "controller dropped off CAN during the window")
        for area, pts, why in [("Battery", 3, "battery sag"), ("roboRIO", 3, "brownout"),
                               ("CAN bus", 2, "CAN bus errors"), ("Radio link", 3, "radio/DS link trouble")]:
            hits = [x for x in e_f if x.area == area]
            if any(near(x.time) for x in hits):
                add("Electrical", pts, f"{why} when the problem started")
            elif hits:
                add("Electrical", 0.5, f"{why} in the window, but not when the problem started "
                                       f"(t={hits[0].time:.1f}s vs {problem_t:.1f}s)")
        if ev["stalls"]:
            add("Mechanical", 4, "high voltage + current with no motion (jam / bind / hard stop)")
        if ev["stator_limit_seconds"] > 1:
            add("Mechanical", 2, "sat at the stator current limit (heavy load)")
        if any("unevenly" in x.message for x in f):
            add("Mechanical", 3, "motors sharing load unevenly")
            add("Programming", 1, "...or a follower configured to fight the leader")
        if (ev.get("max_temp") or 0) >= P.TEMP_WARN_C:
            add("Mechanical", 1, "motor ran hot (sustained load)")
        if any("beyond the soft limits" in x.message for x in f):
            add("Mechanical", 2, "position past the soft limits (zero slipped / not homed)")
            add("Programming", 1, "...or the encoder zero/offset is wrong in code")
        if ev.get("wrong_direction_seconds"):
            add("Programming", 5, "moved opposite the command (inversion / follower alignment)")
        if any("setpoint is 0" in x.message or "placeholders" in x.message for x in f):
            add("Programming", 4, "state setpoint is a 0 placeholder")
        if any("outside the soft limits" in x.message for x in f):
            add("Programming", 4, "setpoint outside the soft limits")
        if ev["no_output_seconds"] > 0.5:
            add("Programming", 3, "far from target but the motor was given ~0 V")
        if ev["state_machine_override_seconds"] > 0.5:
            add("Programming", 1, "state machine guard replaced the requested state")
        if not ev["desired_states_seen"] or ev["desired_states_seen"] == sorted(IDLE_STATE_NAMES & set(ev["desired_states_seen"])):
            add("Programming", 2, "only idle states were ever requested (binding or command not running?)")
        if ev["error_sign_flips"] > 20 or any("oscillating" in x.message for x in f):
            add("Tuning", 3, "oscillates around the target")
        if ev.get("max_overshoot", 0) > 3 * m["tolerance"]:
            add("Tuning", 2, "overshoots the target")
        if ev["unsettled"] and not ev["stalls"] and ev["saturated_seconds"] < 0.5 and not ev["no_output_seconds"]:
            add("Tuning", 2, "misses the target without stalling or saturating (gains too low / kG / kS)")
        if (ev.get("median_settle") or 0) > 1.0:
            add("Tuning", 1, "slow to settle")

        ranked = sorted(((v, k) for k, v in scores.items() if v > 0), reverse=True)
        print("\nLikely causes (heuristic, highest first):")
        if not ranked:
            print("- Nothing stands out for this mechanism in this window.")
        tot = sum(v for v, _ in ranked) or 1
        for v, k in ranked:
            print(f"- {k} ({v / tot * 100:.0f}%): " + "; ".join(reasons[k]))
            print(f"    {CAUSE_FILES[k]}")
        if f:
            print("\nFlags:")
            for x in f:
                print(f"- {x.severity} [{x.category}] {x.message} {log.stamp(x.time)}")
        print("\nCode to read: " + "; ".join(m["files"]) + "\n")


def _finding_key(x: Finding) -> tuple:
    # Ignore the numbers so "stalled 2x (3.1s)" and "stalled 3x (4.0s)" count as the same issue.
    return (x.area, "".join(c for c in x.message if not c.isdigit())[:60])


def cmd_compare(args):
    real, replay = Log(args.log, "real"), Log(args.log, "replay")
    if not any(k.startswith("ReplayOutputs/") for k in real.raw.entries):
        sys.exit("No ReplayOutputs in this log. Pass the _sim log that a replay wrote.")
    r_res, r_find = run_all(real)
    n_res, n_find = run_all(replay)
    r_keys = {_finding_key(x): x for x in r_find}
    n_keys = {_finding_key(x): x for x in n_find}
    print(f"# Replay comparison: {Path(args.log).name}")
    print()
    print("Inputs (sensors) are identical in both; only the code's decisions differ. Replay does "
          "NOT re-simulate physics, so mechanism positions/speeds are the recorded ones. Judge the "
          "change by states, setpoints, readiness, and commands, not by tracking error.")
    print()
    sections = [("Fixed by the code change (only in the original run)", [r_keys[k] for k in r_keys if k not in n_keys]),
                ("New with the code change (only in the replay)", [n_keys[k] for k in n_keys if k not in r_keys]),
                ("Unchanged", [n_keys[k] for k in n_keys if k in r_keys])]
    for title, items in sections:
        print(f"## {title}")
        print()
        for x in items:
            print(f"- {x.severity} [{x.category}] {x.area}: {x.message} {real.stamp(x.time)}")
        if not items:
            print("- none")
        print()
    print("## Decisions")
    print()
    print("| Mechanism | Setpoints by state (original) | Setpoints by state (replay) |")
    print("|---|---|---|")
    for name, m in P.MECHANISMS.items():
        a = r_res[name].get("setpoints_by_state", {})
        b = n_res[name].get("setpoints_by_state", {})
        if a or b:
            print(f"| {m['label']} | {a} | {b} |")
    rs, ns = r_res["shooting"], n_res["shooting"]
    if rs.get("attempts") or ns.get("attempts"):
        print()
        print(f"Shooting ready fraction: original {fmt((rs.get('ready_fraction') or 0) * 100)}% -> "
              f"replay {fmt((ns.get('ready_fraction') or 0) * 100)}%")
    ra = {r[0]: r[2] for r in r_res["states"]}
    na = {r[0]: r[2] for r in n_res["states"]}
    print()
    print("| State | Unconfirmed s (original) | Unconfirmed s (replay) |")
    print("|---|---|---|")
    for k in sorted(set(ra) | set(na)):
        print(f"| {k} | {fmt(ra.get(k))} | {fmt(na.get(k))} |")


def cmd_timeline(args):
    log = Log(args.log, args.outputs, args.ds)
    events = []
    for key, label in [(P.DS_ENABLED[0], "Enabled"), (P.DS_AUTO[0], "Autonomous")]:
        s = log.get(key)
        if s is not None:
            prev = None
            for t, v in zip(s.times, s.values):
                if v != prev:
                    events.append((t, "mode", f"{label} = {v}"))
                prev = v
    for key in log.keys():
        k = key.split("Outputs/", 1)[-1]
        typ = log.raw.type_of(key)
        if k.startswith("StateHandler/") and typ == "string":
            s = log.raw.series(key)
            prev = None
            for t, v in zip(s.times, s.values):
                if v != prev:
                    events.append((t, "state", f"{k[len('StateHandler/'):]} -> {v}"))
                prev = v
        elif k.startswith("Commands/") and typ == "boolean":
            s = log.raw.series(key)
            prev = False
            for t, v in zip(s.times, s.values):
                if v != prev:
                    events.append((t, "command", f"{'START' if v else 'end  '} {k[len('Commands/'):]}"))
                prev = v
        elif key.endswith("Connected") and typ in ("boolean", "boolean[]"):
            s = log.raw.series(key)
            prev = None
            for t, v in zip(s.times, s.values):
                if prev is not None and v != prev and t > 3.0:
                    events.append((t, "fault", f"{key} = {v}"))
                prev = v
        elif k.endswith("/Homed"):
            s = log.raw.series(key)
            for t, v in zip(s.times, s.values):
                events.append((t, "event", f"{k} = {v}"))
    bo = log.get(P.BROWNOUT)
    if bo is not None:
        for a, b in intervals_where(bo, bool, log.end):
            events.append((a, "fault", f"BROWNOUT for {b - a:.2f}s"))
    ds_ev, _ = analyze_ds(log)
    for t, kind, text in ds_ev.get("events", []):
        events.append((t, "ds", f"[{kind}] {text[:200]}"))
    for a, b in ds_ev.get("comms_drops", []):
        events.append((a, "fault", f"DS lost comms for {b - a:.2f}s"))
    events.sort(key=lambda e: e[0])
    kinds = set(args.only.split(",")) if args.only else None
    for t, kind, text in events:
        if args.start is not None and t < args.start:
            continue
        if args.end is not None and t > args.end:
            continue
        if kinds and kind not in kinds:
            continue
        if args.filter and args.filter.lower() not in text.lower():
            continue
        print(f"{t:8.2f}s  {log.mode_at(t):8s}  {kind:7s}  {text}")


def cmd_series(args):
    log = Log(args.log, args.outputs, args.ds)
    series = []
    for key in args.keys:
        s = log.get(key)
        if s is None:
            close = [k for k in log.keys() if key.lower().split("/")[-1] in k.lower()][:10]
            sys.exit(f"'{key}' is not in the log. Similar keys: {close}")
        series.append(s)
    lo = args.start if args.start is not None else 0.0
    hi = args.end if args.end is not None else log.end
    if args.step:
        times = [lo + k * args.step for k in range(int((hi - lo) / args.step) + 1)]
    else:
        times = sorted({t for s in series for t in s.times if lo <= t <= hi})
    times = times[: args.limit]
    print("time_s,mode," + ",".join(args.keys))
    cols = [sample(s, times) for s in series]
    for i, t in enumerate(times):
        cells = []
        for c in cols:
            v = c[i]
            if isinstance(v, dict):
                p = pose2d_tuple(v)
                v = f"({p[0]:.3f} {p[1]:.3f} {p[2]:.1f}deg)" if p else v
            elif isinstance(v, float):
                v = f"{v:.4g}"
            elif isinstance(v, list):
                v = "[" + " ".join(f"{x:.4g}" if isinstance(x, float) else str(x) for x in v) + "]"
            cells.append(str(v))
        print(f"{t:.3f},{log.mode_at(t)}," + ",".join(cells))


def cmd_info(args):
    log = Log(args.log, args.outputs, args.ds)
    print(f"# {Path(args.log).name}")
    print("\n".join(match_header(log)))
    print(f"- Enabled periods: " + ", ".join(f"{a:.1f}-{b:.1f}s" for a, b in log.enabled_intervals))
    keys = log.raw.keys(args.prefix or "")
    print(f"\n{len(keys)} keys" + (f" under {args.prefix}" if args.prefix else "") + ":")
    for k in keys:
        if k.startswith(".schema/") and not args.prefix:
            continue
        e = log.raw.entries[k]
        print(f"  {k}  [{e.type}] {len(e.records)} values")


def cmd_find(args):
    roots = [REPO_ROOT / "logs", Path.home() / "Downloads", Path.home() / "Documents", Path.home() / "Desktop"]
    roots += [Path(p) for p in args.dirs]
    found = []
    for root in roots:
        if root.is_dir():
            for p in root.rglob("*.wpilog"):
                try:
                    found.append((p.stat().st_mtime, p))
                except OSError:
                    pass
    import datetime
    for mtime, p in sorted(found, reverse=True)[: args.limit]:
        when = datetime.datetime.fromtimestamp(mtime).strftime("%Y-%m-%d %H:%M")
        size = p.stat().st_size / 1e6
        print(f"{when}  {size:7.1f} MB  {p}")
    if not found:
        print("No .wpilog files found in: " + ", ".join(str(r) for r in roots))


def cmd_ds(args):
    """Summary of a DS log on its own (no AdvantageKit log needed)."""
    import datetime
    ds = read_dslog(args.dslog)
    start = datetime.datetime.fromtimestamp(ds.start_unix).strftime("%Y-%m-%d %H:%M:%S")
    print(f"# {Path(args.dslog).name}")
    print()
    print(f"- Started {start} (this computer's time zone), {ds.duration:.0f}s long")
    modes: dict[str, float] = {}
    for i in range(len(ds.times)):
        modes[ds.mode(i)] = modes.get(ds.mode(i), 0.0) + DT
    print("- Time by mode: " + ", ".join(f"{k} {v:.0f}s" for k, v in modes.items()))
    en = [i for i in range(len(ds.times)) if ds.enabled(i)]
    if en:
        print(f"- While enabled: trip median {statistics.median(ds.trip_ms[i] for i in en):.1f} ms, "
              f"packet loss max {max(ds.packet_loss[i] for i in en) * 100:.0f}%, battery min "
              f"{min(ds.battery[i] for i in en):.2f} V, brownout cycles {sum(ds.brownout[i] for i in en)}, "
              f"watchdog cycles {sum(ds.watchdog[i] for i in en)}")
    first = next((i for i, c in enumerate(ds.connected) if c), None)
    if first is not None:
        drops = [r for r in runs(ds.times, [not c for c in ds.connected], 0.1) if r[0] > ds.times[first]]
        print(f"- No-reply periods after first connecting: {len(drops)}"
              + (" (" + ", ".join(f"{a:.1f}-{b:.1f}s" for a, b in drops[:8]) + ")" if drops else ""))
    ev_path = Path(args.dslog).with_suffix(".dsevents")
    if ev_path.is_file():
        evs = read_dsevents(ev_path)
        print()
        print("Messages (routine ones hidden):")
        prev = None
        for t, text in evs.events:
            kind = ds_event_kind(text)
            if kind is None or text == prev:
                continue
            prev = text
            print(f"  {t:8.1f}s [{kind}] {text[:180]}")


def find_usb_roots() -> list[Path]:
    """Mounted drives that look like the robot's log stick (a logs/ or dslogs/ folder at the root)."""
    if platform.system() == "Windows":
        roots = [Path(f"{letter}:/") for letter in string.ascii_uppercase[3:]]  # D: and up
    elif platform.system() == "Darwin":
        roots = list(Path("/Volumes").glob("*"))
    else:
        roots = list(Path("/media").glob("*/*")) + list(Path("/run/media").glob("*/*"))
    found = []
    for r in roots:
        try:
            if (r / "logs").is_dir() or (r / "dslogs").is_dir():
                found.append(r)
        except OSError:
            continue  # empty card reader, locked drive
    return found


def _copy_if_new(src: Path, dest: Path) -> bool:
    """Copies src to dest unless an identical-size copy is already there. Returns True if copied."""
    if dest.exists() and dest.stat().st_size == src.stat().st_size:
        return False
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dest)
    return True


def cmd_ingest(args):
    if args.usb:
        usb = Path(args.usb)
    else:
        roots = find_usb_roots()
        if not roots:
            sys.exit("No log stick found (looked for a drive with a logs/ or dslogs/ folder). "
                     "Plug it in or pass its path, e.g. ingest E:/")
        if len(roots) > 1:
            sys.exit("Several possible log sticks: " + ", ".join(map(str, roots)) + ". Pass one.")
        usb = roots[0]
    dest = Path(args.dest) if args.dest else REPO_ROOT / "logs" / "matches"
    akit = sorted(p for p in (usb / "logs").glob("*.wpilog") if not p.stem.endswith("_sim")) \
        if (usb / "logs").is_dir() else []
    ds_dir = usb / "dslogs"
    dslogs = sorted(ds_dir.glob("*.dslog")) if ds_dir.is_dir() else []
    print(f"Stick {usb}: {len(akit)} AdvantageKit logs, {len(dslogs)} DS logs -> {dest}")

    # Time window of every DS log, read once.
    ds_windows = []
    for p in dslogs:
        try:
            d = read_dslog(p)
            ds_windows.append((p, d.start_unix, d.end_unix))
        except (ValueError, OSError) as e:
            print(f"  skipping unreadable DS log {p.name}: {e}")
    used_ds: set = set()
    rows, new_logs = [], []
    for p in akit:
        try:
            raw = WpiLog.open(p)
        except (ValueError, OSError) as e:
            print(f"  skipping unreadable {p.name}: {e}")
            continue
        folder = dest / p.stem
        copied = _copy_if_new(p, folder / p.name)
        enabled = raw.series("DriverStation/Enabled")
        en_s = sum(b - a for a, b in intervals_where(enabled, bool, raw.duration)) if enabled else 0.0
        epoch0 = akit_epoch_offset(raw)
        paired = None
        if epoch0 is not None:
            best = 0.0
            for dp, a, b in ds_windows:
                overlap = min(epoch0 + raw.duration, b) - max(epoch0, a)
                if overlap > best:
                    best, paired = overlap, dp
        if paired is not None:
            used_ds.add(paired)
            copied |= _copy_if_new(paired, folder / paired.name)
            ev = paired.with_suffix(".dsevents")
            if ev.is_file():
                _copy_if_new(ev, folder / ev.name)
        rows.append((p.stem, raw.duration, en_s, paired.name if paired else "-", copied))
        if copied:
            new_logs.append((folder / p.name, en_s))
    # DS logs with no AdvantageKit log (robot code not running, stick out, etc.) are kept too.
    for dp, _, _ in ds_windows:
        if dp not in used_ds:
            _copy_if_new(dp, dest / "_ds_only" / dp.name)
            ev = dp.with_suffix(".dsevents")
            if ev.is_file():
                _copy_if_new(ev, dest / "_ds_only" / ev.name)

    print("\n| Log | Length s | Enabled s | DS log | New |\n|---|---|---|---|---|")
    for name, dur, en_s, ds_name, copied in rows:
        print(f"| {name} | {dur:.0f} | {en_s:.0f} | {ds_name} | {'yes' if copied else ''} |")

    if args.report:
        todo = [(lp, en) for lp, en in new_logs if en >= args.min_enabled] if not args.all_reports else \
            [(dest / r[0] / f"{r[0]}.wpilog", r[2]) for r in rows if r[2] >= args.min_enabled]
        for lp, _ in todo:
            out = lp.with_name("report.md")
            cmd_report(argparse.Namespace(log=str(lp), out=str(out), outputs="auto", ds=None))
        skipped = len([1 for _, en in new_logs if en < args.min_enabled])
        if skipped:
            print(f"(No report for {skipped} new log(s) with under {args.min_enabled:.0f}s enabled: "
                  "pit restarts and code redeploys.)")
        write_index(dest)


def write_index(dest: Path):
    """INDEX.md: one row per ingested match with its finding counts, newest first."""
    rows = []
    for folder in sorted((d for d in dest.iterdir() if d.is_dir() and not d.name.startswith("_")),
                         reverse=True):
        rep = folder / "report.md"
        if not rep.is_file():
            continue
        text = rep.read_text(encoding="utf-8")
        crit, warn = text.count("**CRITICAL**"), text.count("**WARN**")
        first = next((ln[2:] for ln in text.splitlines() if ln.startswith("- **CRITICAL**")
                      or ln.startswith("- **WARN**")), "")
        has_ds = "Driver Station log:" in text
        rows.append(f"| [{folder.name}]({folder.name}/report.md) | {crit} | {warn} | "
                    f"{'yes' if has_ds else 'no'} | {first[:120]} |")
    lines = ["# Match log index", "", "| Match | Critical | Warn | DS log | Top finding |",
             "|---|---|---|---|---|"] + rows
    (dest / "INDEX.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Index written to {dest / 'INDEX.md'}")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def add_log(p):
        p.add_argument("log")
        p.add_argument("--ds", help="Driver Station .dslog to pair with (default: find one next to "
                                    "the log by time; 'none' to skip)")
        p.add_argument("--outputs", choices=["auto", "real", "replay"], default="auto",
                       help="which decisions to analyze in a replay (_sim) log: the original "
                            "robot's (real) or the current code's (replay). auto = replay if present")

    def window(p):
        p.add_argument("--from", dest="start", type=float, help="start time, seconds since log start")
        p.add_argument("--to", dest="end", type=float, help="end time, seconds since log start")

    p = sub.add_parser("find", help="list .wpilog files, newest first")
    p.add_argument("dirs", nargs="*", help="extra folders to search")
    p.add_argument("--limit", type=int, default=15)
    p.set_defaults(fn=cmd_find)

    p = sub.add_parser("ingest", help="copy logs off the robot's USB stick, pair DS logs, write reports")
    p.add_argument("usb", nargs="?", help="stick path, e.g. E:/ (default: find it)")
    p.add_argument("--dest", help="where to put matches (default: <repo>/logs/matches)")
    p.add_argument("--report", action="store_true", help="write report.md for each new match + INDEX.md")
    p.add_argument("--all-reports", action="store_true", help="with --report: redo every match, not just new")
    p.add_argument("--min-enabled", type=float, default=5.0, help="skip reports for logs enabled less than this (s)")
    p.set_defaults(fn=cmd_ingest)

    p = sub.add_parser("ds", help="summarize a Driver Station .dslog (+ .dsevents) on its own")
    p.add_argument("dslog")
    p.set_defaults(fn=cmd_ds)

    p = sub.add_parser("info", help="match info, enabled periods, and keys")
    add_log(p)
    p.add_argument("--prefix", help="only keys starting with this, e.g. Shooter/")
    p.set_defaults(fn=cmd_info)

    p = sub.add_parser("report", help="full markdown health report")
    add_log(p)
    p.add_argument("--out", help="write the report to this file instead of printing it")
    p.set_defaults(fn=cmd_report)

    p = sub.add_parser("diagnose", help="evidence and ranked causes for one mechanism (or all)")
    add_log(p)
    p.add_argument("mech", nargs="?", help="one of: " + ", ".join(P.MECHANISMS))
    window(p)
    p.set_defaults(fn=cmd_diagnose)

    p = sub.add_parser("timeline", help="mode, state, command, and fault events in order")
    add_log(p)
    window(p)
    p.add_argument("--only", help="comma list of kinds: mode,state,command,fault,event,ds")
    p.add_argument("--filter", help="only events containing this text")
    p.set_defaults(fn=cmd_timeline)

    p = sub.add_parser("compare", help="replay (_sim) log: what the code change changed")
    p.add_argument("log", help="a replay output log containing both RealOutputs and ReplayOutputs")
    p.set_defaults(fn=cmd_compare)

    p = sub.add_parser("series", help="print key values over time as CSV")
    add_log(p)
    p.add_argument("keys", nargs="+")
    window(p)
    p.add_argument("--step", type=float, help="resample every N seconds (default: every change)")
    p.add_argument("--limit", type=int, default=2000, help="max rows")
    p.set_defaults(fn=cmd_series)

    args = ap.parse_args(argv)
    if getattr(args, "log", None) and not os.path.isfile(args.log):
        sys.exit(f"Log not found: {args.log}")
    args.fn(args)


if __name__ == "__main__":
    main()
