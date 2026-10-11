"""Reader for FRC Driver Station logs (.dslog cycle data and .dsevents messages), version 4.

Ported from AdvantageScope's DSLogReader/DSEventsReader (BSD, Littleton Robotics), which credits
https://github.com/orangelight/DSLOG-Reader. All numbers are big-endian. Each .dslog record is one
20 ms DS cycle: trip time, packet loss, battery, roboRIO CPU, status bits, CAN utilization, Wi-Fi,
and power distribution channel currents.

Times: `start_unix` is real UTC unix time, so a DS log can be lined up with an AdvantageKit log's
SystemStats/EpochTimeMicros. Record times are seconds since the DS log started.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass, field
from pathlib import Path

_LABVIEW_EPOCH_OFFSET = 2082844800  # seconds from 1904-01-01 to 1970-01-01 (UTC)
_PERIOD = 0.02
_PD_REV, _PD_CTRE = 33, 25


def _lv_time(data: bytes, offset: int) -> float:
    seconds, fraction = struct.unpack_from(">qQ", data, offset)
    return seconds - _LABVIEW_EPOCH_OFFSET + fraction / 2**64


@dataclass
class DsLog:
    path: str
    start_unix: float
    # Parallel lists, one entry per 20 ms DS cycle.
    times: list = field(default_factory=list)
    trip_ms: list = field(default_factory=list)
    packet_loss: list = field(default_factory=list)  # 0-1
    battery: list = field(default_factory=list)
    cpu: list = field(default_factory=list)  # 0-1
    can_util: list = field(default_factory=list)  # 0-1
    wifi_db: list = field(default_factory=list)
    wifi_mb: list = field(default_factory=list)
    brownout: list = field(default_factory=list)
    watchdog: list = field(default_factory=list)
    ds_disabled: list = field(default_factory=list)
    ds_teleop: list = field(default_factory=list)
    robot_disabled: list = field(default_factory=list)
    robot_auto: list = field(default_factory=list)
    robot_teleop: list = field(default_factory=list)
    connected: list = field(default_factory=list)  # False when no packet came back (trip byte 255)
    pd_type: str = "none"  # "rev", "ctre", or "none"
    pd_currents: list = field(default_factory=list)  # per cycle: list of channel amps

    def enabled(self, i: int) -> bool:
        # The robot-reported mode bits stay idle in some sessions (sim, no code), so use the DS's
        # own disabled bit, and only count cycles where the robot is actually answering.
        return self.connected[i] and not self.ds_disabled[i]

    def mode(self, i: int) -> str:
        if not self.connected[i]:
            return "no-comms"
        if self.ds_disabled[i]:
            return "disabled"
        if self.ds_teleop[i]:
            return "teleop"
        return "auto"

    @property
    def duration(self) -> float:
        return len(self.times) * _PERIOD

    @property
    def end_unix(self) -> float:
        return self.start_unix + self.duration


@dataclass
class DsEvents:
    path: str
    start_unix: float
    events: list = field(default_factory=list)  # (seconds since start, text)


def read_dslog(path) -> DsLog:
    data = Path(path).read_bytes()
    if len(data) < 20 or struct.unpack_from(">i", data, 0)[0] != 4:
        raise ValueError(f"{path}: not a version 4 .dslog")
    log = DsLog(str(path), _lv_time(data, 4))
    pos, t, last_batt, n = 20, 0.0, 0.0, len(data)
    while pos + 14 <= n:
        trip, loss, batt, cpu, mask, can, wdb, wmb = struct.unpack_from(">BbHBBBBH", data, pos)
        batt /= 256
        if batt > 20:  # the last record sometimes holds garbage voltage
            batt = last_batt
        last_batt = batt
        pos += 10
        pd_id = data[pos + 3]
        pos += 4
        currents = []
        if pd_id == _PD_REV:
            if pos + 33 > n:
                break
            bits = _bits(data[pos + 1 : pos + 28])
            for i in range(20):
                start = (i // 3) * 32 + (i % 3) * 10
                currents.append(_bits_value(bits, start, 10) / 8)
            currents += [data[pos + 28 + i] / 16 for i in range(4)]
            pos += 33
            log.pd_type = "rev"
        elif pd_id == _PD_CTRE:
            if pos + 25 > n:
                break
            bits = _bits(data[pos + 1 : pos + 22])
            for i in range(16):
                start = (i // 6) * 64 + (i % 6) * 10
                # AdvantageScope reads 8 of the 10 bits here; matched so values agree with it.
                currents.append(_bits_value(bits, start, 8) / 8)
            pos += 25
            log.pd_type = "ctre"
        log.times.append(t)
        log.connected.append(trip != 255)
        log.trip_ms.append(trip * 0.5)
        log.packet_loss.append(min(max(loss * 4 * 0.01, 0.0), 1.0))
        log.battery.append(batt)
        log.cpu.append(cpu * 0.5 * 0.01)
        log.can_util.append(can * 0.5 * 0.01)
        log.wifi_db.append(wdb * 0.5)
        log.wifi_mb.append(wmb / 256)
        # Status bits are active-low.
        log.brownout.append(not mask & 0x80)
        log.watchdog.append(not mask & 0x40)
        log.ds_teleop.append(not mask & 0x20)
        log.ds_disabled.append(not mask & 0x08)
        log.robot_teleop.append(not mask & 0x04)
        log.robot_auto.append(not mask & 0x02)
        log.robot_disabled.append(not mask & 0x01)
        log.pd_currents.append(currents)
        t += _PERIOD
    return log


def read_dslog_window(path) -> tuple:
    """(start_unix, end_unix) of a .dslog, stepping over records without decoding them. Same
    record count as read_dslog, so it agrees with DsLog.start_unix/end_unix."""
    data = Path(path).read_bytes()
    if len(data) < 20 or struct.unpack_from(">i", data, 0)[0] != 4:
        raise ValueError(f"{path}: not a version 4 .dslog")
    start = _lv_time(data, 4)
    pos, count, n = 20, 0, len(data)
    while pos + 14 <= n:
        pd_id = data[pos + 13]
        pos += 14
        size = 33 if pd_id == _PD_REV else 25 if pd_id == _PD_CTRE else 0
        if pos + size > n:
            break
        pos += size
        count += 1
    return start, start + count * _PERIOD


def read_dsevents(path) -> DsEvents:
    data = Path(path).read_bytes()
    if len(data) < 20 or struct.unpack_from(">i", data, 0)[0] != 4:
        raise ValueError(f"{path}: not a version 4 .dsevents")
    start = _lv_time(data, 4)
    out = DsEvents(str(path), start)
    pos, n = 20, len(data)
    while pos + 20 <= n:
        ts = _lv_time(data, pos)
        (length,) = struct.unpack_from(">i", data, pos + 16)
        pos += 20
        if length < 0 or pos + length > n:
            break
        text = _clean_event(data[pos : pos + length].decode("utf-8", "replace"))
        pos += length
        out.events.append((ts - start, text))
    return out


def _clean_event(text: str) -> str:
    for tag in ("<TagVersion>", "<time>", "<count>", "<flags>", "<Code>", "<location>", "<stack>"):
        while tag in text:
            i = text.index(tag)
            j = text.find("<", i + 1)
            text = text[:i] + (text[j:] if j >= 0 else "")
    return text.replace("<message> ", "").replace("<details> ", "").strip()


def _bits(chunk: bytes) -> list:
    return [(byte >> i) & 1 for byte in chunk for i in range(8)]


def _bits_value(bits: list, start: int, width: int) -> int:
    return sum(bits[start + i] << i for i in range(width) if start + i < len(bits))
