"""Self-test: writes a synthetic AdvantageKit-style .wpilog with known faults and checks that the
parser decodes it and the analyzer reports each fault. Run: python test_analyze.py
"""

from __future__ import annotations

import contextlib
import io
import math
import struct
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import analyze  # noqa: E402
from wpilog import WpiLog  # noqa: E402


class WpiLogWriter:
    """Minimal WPILOG writer following the spec (always 4-byte id/size, 8-byte timestamp)."""

    def __init__(self):
        header = b"AdvantageKit"  # AdvantageKit replay rejects logs without this extra header
        self.buf = bytearray(b"WPILOG" + struct.pack("<HI", 0x0100, len(header)) + header)
        self.ids: dict[str, tuple[int, str]] = {}

    def _record(self, entry_id: int, ts_us: int, payload: bytes):
        self.buf.append(0b0111_1111)  # id 4 bytes, size 4 bytes, timestamp 8 bytes
        self.buf += struct.pack("<IIQ", entry_id, len(payload), ts_us) + payload

    def start(self, name: str, type_: str, ts_us: int = 0):
        entry_id = len(self.ids) + 1
        self.ids[name] = (entry_id, type_)
        n, t, m = name.encode(), type_.encode(), b'{"source":"AdvantageKit"}'
        payload = (bytes([0]) + struct.pack("<I", entry_id) + struct.pack("<I", len(n)) + n
                   + struct.pack("<I", len(t)) + t + struct.pack("<I", len(m)) + m)
        self._record(0, ts_us, payload)

    def put(self, name: str, ts: float, value):
        if name not in self.ids:
            raise KeyError(name)
        entry_id, type_ = self.ids[name]
        if type_ == "boolean":
            payload = bytes([1 if value else 0])
        elif type_ == "double":
            payload = struct.pack("<d", value)
        elif type_ == "int64":
            payload = struct.pack("<q", value)
        elif type_ == "string" or type_ == "structschema":
            payload = value.encode()
        elif type_ == "double[]":
            payload = struct.pack(f"<{len(value)}d", *value)
        elif type_ == "boolean[]":
            payload = bytes(1 if v else 0 for v in value)
        elif type_ == "string[]":
            payload = struct.pack("<I", len(value))
            for s in value:
                payload += struct.pack("<I", len(s.encode())) + s.encode()
        elif type_ == "struct:Pose2d":
            payload = struct.pack("<ddd", *value)
        else:
            raise ValueError(type_)
        self._record(entry_id, int(ts * 1e6), payload)


def build_log(path: Path, epoch: bool = True, robot_mode: str = "REAL"):
    """45 s log: 2 s disabled, 40 s teleop enabled, 3 s disabled. Faults are injected on purpose."""
    w = WpiLogWriter()
    path.parent.mkdir(parents=True, exist_ok=True)
    A = "/RealOutputs/"
    schema = {
        "/.schema/struct:Translation2d": "double x;double y",
        "/.schema/struct:Rotation2d": "double value",
        "/.schema/struct:Pose2d": "Translation2d translation;Rotation2d rotation",
    }
    for k, v in schema.items():
        w.start(k, "structschema")
        w.put(k, 0, v)
    keys = {
        "/Timestamp": "int64",  # AdvantageKit writes this every cycle; replay steps on it
        "/SystemStats/EpochTimeMicros": "int64",  # real UTC time, used to line up DS logs
        "/RealMetadata/RobotMode": "string",
        "/DriverStation/Enabled": "boolean", "/DriverStation/Autonomous": "boolean",
        "/DriverStation/DSAttached": "boolean",  # WPILib only reports enabled with a DS attached
        "/SystemStats/BatteryVoltage": "double", "/SystemStats/BrownedOut": "boolean",
        "/Swerve/Pose": "struct:Pose2d",
        "/Shooter/HoodPositionDegrees": "double", "/Shooter/HoodVelocityDegreesPerSecond": "double",
        "/Shooter/HoodAppliedVolts": "double", "/Shooter/HoodStatorCurrentAmps": "double",
        "/Shooter/HoodSupplyCurrentAmps": "double", "/Shooter/HoodTemperatureCelsius": "double",
        "/Shooter/HoodConnected": "boolean",
        A + "Shooter/Hood/SetpointDegrees": "double", A + "Shooter/Hood/AppliedState": "string",
        A + "StateHandler/Shooter/DesiredHoodState": "string",
        A + "StateHandler/Shooter/CurrentHoodState": "string",
        "/Shooter/DrumVelocityRpm": "double", "/Shooter/DrumStatorCurrentAmps": "double[]",
        "/Shooter/DrumSupplyCurrentAmps": "double[]", "/Shooter/DrumAppliedVolts": "double[]",
        "/Shooter/DrumTemperatureCelsius": "double[]", "/Shooter/DrumConnected": "boolean[]",
        A + "Shooter/Drum/SetpointRpm": "double", A + "Shooter/Drum/AppliedState": "string",
        "/Rollers/FloorVelocityRpm": "double", "/Rollers/FloorAppliedVolts": "double",
        "/Rollers/FloorStatorCurrentAmps": "double", "/Rollers/FloorSupplyCurrentAmps": "double",
        "/Rollers/FloorConnected": "boolean",
        A + "Rollers/Floor/SetpointRpm": "double", A + "Rollers/Floor/AppliedState": "string",
        A + "Commands/ShootGamePieces": "boolean", A + "Shooter/ReadyToShoot": "boolean",
        A + "Alerts/errors": "string[]",
    }
    for k, t in keys.items():
        w.start(k, t)

    hood = 0.0
    for i in range(int(45 / 0.02)):
        t = i * 0.02
        enabled = 2.0 <= t < 42.0
        w.put("/Timestamp", t, int(t * 1e6))
        if epoch:
            w.put("/SystemStats/EpochTimeMicros", t, int((EPOCH0 + t) * 1e6))
        if i == 0:
            w.put("/RealMetadata/RobotMode", t, robot_mode)
        w.put("/DriverStation/Enabled", t, enabled)
        w.put("/DriverStation/Autonomous", t, False)
        w.put("/DriverStation/DSAttached", t, True)
        # Brownout at 30-30.3 s with the battery sagging.
        sag = 30.0 <= t < 30.3
        w.put("/SystemStats/BatteryVoltage", t, 6.5 if sag else 12.3)
        w.put("/SystemStats/BrownedOut", t, sag)
        w.put("/Swerve/Pose", t, (1.0 + 0.01 * i, 2.0, math.radians(90)))

        # Hood: asked for 20 deg from 5 s, but jams at 12 deg (stall: high volts+current, no motion).
        hood_sp = 20.0 if t >= 5.0 else 0.0
        if enabled and t >= 5.0:
            hood = min(hood + 0.8, 12.0)
        stuck = enabled and hood >= 12.0 and hood_sp == 20.0
        w.put(A + "Shooter/Hood/SetpointDegrees", t, hood_sp)
        w.put(A + "Shooter/Hood/AppliedState", t, "SHOOTING" if hood_sp else "STOWED")
        w.put(A + "StateHandler/Shooter/DesiredHoodState", t, "SHOOTING" if hood_sp else "STOWED")
        w.put(A + "StateHandler/Shooter/CurrentHoodState", t, "STOWED")
        w.put("/Shooter/HoodPositionDegrees", t, hood)
        w.put("/Shooter/HoodVelocityDegreesPerSecond", t, 0.0 if stuck or not enabled else 40.0)
        w.put("/Shooter/HoodAppliedVolts", t, 6.0 if stuck else 0.0)
        w.put("/Shooter/HoodStatorCurrentAmps", t, 35.0 if stuck else 1.0)
        w.put("/Shooter/HoodSupplyCurrentAmps", t, 8.0 if stuck else 0.5)
        w.put("/Shooter/HoodTemperatureCelsius", t, 30.0 + t)
        w.put("/Shooter/HoodConnected", t, True)

        # Drum: 3000 RPM, follower two pulls 3x the current of the others; follower three drops off CAN.
        drum_sp = 3000.0 if enabled else 0.0
        drum = min(3000.0, max(0.0, (t - 2.0) * 1500.0)) if enabled else 0.0
        w.put(A + "Shooter/Drum/SetpointRpm", t, drum_sp)
        w.put(A + "Shooter/Drum/AppliedState", t, "SHOOTING" if enabled else "IDLE")
        w.put("/Shooter/DrumVelocityRpm", t, drum)
        w.put("/Shooter/DrumStatorCurrentAmps", t, [20.0, 20.0, 60.0, 20.0] if enabled else [0.0] * 4)
        w.put("/Shooter/DrumSupplyCurrentAmps", t, [10.0] * 4)
        w.put("/Shooter/DrumAppliedVolts", t, [6.0, 6.0, -6.0, -6.0] if enabled else [0.0] * 4)
        w.put("/Shooter/DrumTemperatureCelsius", t, [40.0, 40.0, 75.0, 40.0])
        w.put("/Shooter/DrumConnected", t, [True, True, True, not (20.0 <= t < 21.0)])

        # Floor roller: FEED state applied with a 0 RPM placeholder setpoint.
        w.put(A + "Rollers/Floor/SetpointRpm", t, 0.0)
        w.put(A + "Rollers/Floor/AppliedState", t, "FEED" if enabled else "REST")
        w.put("/Rollers/FloorVelocityRpm", t, 0.0)
        w.put("/Rollers/FloorAppliedVolts", t, 0.0)
        w.put("/Rollers/FloorStatorCurrentAmps", t, 0.0)
        w.put("/Rollers/FloorSupplyCurrentAmps", t, 0.0)
        w.put("/Rollers/FloorConnected", t, True)

        w.put(A + "Commands/ShootGamePieces", t, enabled and 10.0 <= t < 15.0)
        w.put(A + "Shooter/ReadyToShoot", t, False)
        w.put(A + "Alerts/errors", t, ["Shooter drum follower three motor disconnected"]
              if 20.0 <= t < 21.0 else [])
    path.write_bytes(bytes(w.buf))


EPOCH0 = 1_790_000_000.0  # unix time of the synthetic AdvantageKit log's t=0
LV_OFFSET = 2_082_844_800  # LabVIEW epoch (1904) to unix epoch


def _lv(unix: float) -> bytes:
    seconds = int(unix // 1) + LV_OFFSET
    return struct.pack(">qQ", seconds, int((unix % 1) * 2**64))


def build_ds_files(folder: Path, start_unix: float, length: float = 46.0, stem: str = "ds",
                   epoch0: float = EPOCH0):
    """A .dslog/.dsevents pair starting at start_unix: enabled 2-42 s on the AdvantageKit clock,
    40% packet loss at 25-27 s, no comms at 33-34 s, REV PDH channel 5 drawing 30 A while enabled."""
    folder.mkdir(parents=True, exist_ok=True)
    buf = bytearray(struct.pack(">i", 4) + _lv(start_unix))
    n = int(length / 0.02)
    for i in range(n):
        t = start_unix + i * 0.02 - epoch0  # time on the AdvantageKit clock
        enabled = 2.0 <= t < 42.0
        no_comms = 33.0 <= t < 34.0
        trip = 255 if no_comms else 4  # 2 ms
        loss = 10 if 25.0 <= t < 27.0 else 0  # x4 % -> 40%
        mask = 0xFF
        if not enabled:
            mask &= ~0x08  # DS disabled bit is active-low
        else:
            mask &= ~0x20  # teleop
        buf += struct.pack(">BbHBBBBH", trip, loss, int(12.5 * 256), 60, mask, 40, 80, 2 * 256)
        buf += bytes([0, 0, 0, 33])  # PD header, type 33 = REV PDH
        bits = [0] * 216
        amps = [0.0] * 20
        if enabled:
            amps[5] = 30.0
        for ch, a in enumerate(amps):
            start, value = (ch // 3) * 32 + (ch % 3) * 10, int(a * 8)
            for b in range(10):
                bits[start + b] = (value >> b) & 1
        packed = bytes(sum(bits[k * 8 + b] << b for b in range(8)) for k in range(27))
        buf += bytes([1]) + packed + bytes(4) + bytes([0])
    (folder / f"{stem}.dslog").write_bytes(bytes(buf))

    ev = bytearray(struct.pack(">i", 4) + _lv(start_unix))
    for t, text in [(10.0, "<TagVersion>1 <time> 00:10.000 <message> Info roboRIO: Disk Free 100 MB"),
                    (12.0, "<TagVersion>1 <time> 00:12.000 <message> Joystick 1 was unplugged"),
                    (33.0, "<TagVersion>1 <time> 00:33.000 <message> FRC: The Driver Station has lost communication with the robot.")]:
        body = text.encode()
        ev += _lv(epoch0 + t) + struct.pack(">i", len(body)) + body
    (folder / f"{stem}.dsevents").write_bytes(bytes(ev))


class AnalyzerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.path = Path(cls.tmp.name) / "synthetic.wpilog"
        build_log(cls.path)
        build_ds_files(Path(cls.tmp.name) / "dslogs", EPOCH0 - 1.0)

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def run_cmd(self, *argv) -> str:
        out = io.StringIO()
        with contextlib.redirect_stdout(out):
            analyze.main(list(argv))
        return out.getvalue()

    def test_parser_decodes_every_type(self):
        log = WpiLog.open(self.path)
        self.assertAlmostEqual(log.duration, 44.98, places=1)
        pose = log.series("Swerve/Pose").values[0]
        self.assertAlmostEqual(pose["translation"]["x"], 1.0)
        self.assertAlmostEqual(pose["rotation"]["value"], math.radians(90))
        self.assertEqual(log.series("/Shooter/DrumAppliedVolts").values[100], [6.0, 6.0, -6.0, -6.0])
        self.assertEqual(log.series("AdvantageKit/RealOutputs/Shooter/Hood/AppliedState").values[0], "STOWED")
        alerts = log.series("RealOutputs/Alerts/errors")
        self.assertIn("Shooter drum follower three motor disconnected", alerts.at(20.5))

    def test_report_flags_injected_faults(self):
        text = self.run_cmd("report", str(self.path))
        self.assertIn("stalled", text)  # hood jam
        self.assertIn("unevenly", text)  # drum follower two
        self.assertIn("follower three: motor disconnected", text)
        self.assertIn("browned out", text)
        self.assertIn("state FEED was applied but its setpoint is 0", text)
        self.assertIn("never became ready", text)

    def test_diagnose_ranks_mechanical_for_jammed_hood(self):
        text = self.run_cmd("diagnose", str(self.path), "hood")
        causes = text.split("Likely causes")[1]
        first = causes.strip().splitlines()[1]
        self.assertIn("Mechanical", first)

    def test_diagnose_drum_sees_electrical_disconnect(self):
        text = self.run_cmd("diagnose", str(self.path), "drum", "--from", "19", "--to", "22")
        self.assertIn("DISCONNECT", text)
        self.assertIn("Electrical", text.split("Likely causes")[1].strip().splitlines()[1])

    def test_timeline_and_series(self):
        tl = self.run_cmd("timeline", str(self.path), "--only", "fault")
        self.assertIn("BROWNOUT", tl)
        csv = self.run_cmd("series", str(self.path), "Shooter/HoodPositionDegrees", "Swerve/Pose",
                           "--from", "10", "--to", "10.1", "--step", "0.05")
        self.assertIn("12", csv.splitlines()[1])
        self.assertIn("deg)", csv)


    def test_ds_log_is_paired_and_analyzed(self):
        text = self.run_cmd("report", str(self.path))
        self.assertIn("Driver Station log: ds.dslog", text)
        self.assertIn("lost communication with the robot 1x (at 33.0s)", text)  # cycle gap + message merged
        self.assertIn("packet loss at or above 20%", text)
        self.assertIn("controller problem", text)
        self.assertIn("| 5 | (unmapped) | 30.0 |", text)
        self.assertNotIn("Disk Free", text)  # routine DS noise is filtered
        tl = self.run_cmd("timeline", str(self.path), "--only", "ds,fault")
        self.assertRegex(tl, r"33\.0\ds.*lost communication")
        no_ds = self.run_cmd("report", str(self.path), "--ds", "none")
        self.assertNotIn("Driver Station log:", no_ds)

    def test_ingest_from_usb_stick(self):
        usb = Path(self.tmp.name) / "usb"
        name = "akit_26-09-21_14-00-00_njfla_q12"
        (usb / "logs").mkdir(parents=True)
        (usb / "logs" / f"{name}.wpilog").write_bytes(self.path.read_bytes())
        build_ds_files(usb / "dslogs", EPOCH0 - 1.0, stem="2026_09_21 10_00_00 Mon")
        build_ds_files(usb / "dslogs", EPOCH0 + 5000, stem="2026_09_21 11_30_00 Mon")  # no robot log
        dest = Path(self.tmp.name) / "matches"
        out = self.run_cmd("ingest", str(usb), "--dest", str(dest), "--report")
        self.assertIn("2026_09_21 10_00_00 Mon.dslog", out)
        match = dest / name
        self.assertTrue((match / "2026_09_21 10_00_00 Mon.dsevents").is_file())
        self.assertIn("Driver Station log:", (match / "report.md").read_text(encoding="utf-8"))
        self.assertTrue((dest / "_ds_only" / "2026_09_21 11_30_00 Mon.dslog").is_file())
        self.assertIn(name, (dest / "INDEX.md").read_text(encoding="utf-8"))
        again = self.run_cmd("ingest", str(usb), "--dest", str(dest))
        self.assertNotIn("| yes |", again)  # nothing new the second time


    def test_sim_log_pairs_by_filename_time(self):
        """Sim logs may lack a valid roboRIO clock; pairing then uses the local time in the name."""
        import datetime
        folder = Path(self.tmp.name) / "simpair"
        start = datetime.datetime(2026, 9, 20, 16, 30, 0)
        name = "akit_" + start.strftime("%y-%m-%d_%H-%M-%S") + ".wpilog"
        build_log(folder / name, epoch=False, robot_mode="SIM")
        build_ds_files(folder / "dslogs", start.timestamp() - 2.0, stem="2026_09_20 16_29_58 Sun",
                       epoch0=start.timestamp())
        text = self.run_cmd("report", str(folder / name))
        self.assertIn("Driver Station log: 2026_09_20 16_29_58 Sun.dslog", text)

    def test_pd_channel_names_come_from_robot_overview(self):
        import robot_model
        repo = Path(self.tmp.name) / "repo"
        (repo / "docs").mkdir(parents=True)
        table = [
            "| PDH channel | Device | Breaker (A) |",
            "| --- | --- | --- |",
            "| 0 | FL drive | 40 |",
            "| 1 |  |  |",
            "| 10 | Drum leader | 40 |",
            "",
            "After the table.",
        ]
        (repo / "docs" / "ROBOT_OVERVIEW.md").write_text("\n".join(table), encoding="utf-8")
        self.assertEqual(robot_model.parse_pd_channels(repo), {0: "FL drive", 10: "Drum leader"})


if __name__ == "__main__":
    unittest.main(verbosity=2)
