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


def build_log(path: Path):
    """45 s log: 2 s disabled, 40 s teleop enabled, 3 s disabled. Faults are injected on purpose."""
    w = WpiLogWriter()
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


class AnalyzerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.path = Path(cls.tmp.name) / "synthetic.wpilog"
        build_log(cls.path)

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


if __name__ == "__main__":
    unittest.main(verbosity=2)
