"""Works out what the analyzer needs to know about the robot from sources that are always current,
so there is nothing to keep in sync by hand:

- The log: which mechanisms exist and their keys. Every mechanism follows the codebase's naming:
    outputs  RealOutputs/<Subsystem>/<Mech>/Setpoint<Degrees|Rpm|Volts>, .../<Mech>/AppliedState
    inputs   <Subsystem>/<Mech>PositionDegrees | VelocityRpm | AppliedVolts | StatorCurrentAmps ...
    states   RealOutputs/StateHandler/<Subsystem>/Desired<Mech>State, Current<Mech>State
- Constants.java (the checked-out branch): tolerances, current limits, soft limits, gear ratios.
  <Subsystem>Constants holds <MECH>_CONFIG (or its only TalonFXConfiguration) and a *TOLERANCE*.
- docs/ROBOT_OVERVIEW.md: the power distribution channel table (wiring isn't in the code).

If a value can't be found, a sensible default is used and listed in `notes`, which the report shows.
"""

from __future__ import annotations

import ast
import math
import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[4]
SRC = "src/main/java/frc/robot"

# ---- AdvantageKit's own keys: the same on every AdvantageKit robot ----------------------------
# Fallbacks cover plain WPILib DataLogManager logs ("DS:enabled").
DS_ENABLED = ["DriverStation/Enabled", "DS:enabled"]
DS_AUTO = ["DriverStation/Autonomous", "DS:autonomous"]
DS_TEST = ["DriverStation/Test", "DS:test"]
BATTERY = ["SystemStats/BatteryVoltage"]
BROWNOUT = "SystemStats/BrownedOut"
COMMS_DISABLE = "SystemStats/CommsDisableCount"
CAN_UTIL = "SystemStats/CANBus/Utilization"
CAN_OFF = "SystemStats/CANBus/OffCount"
CAN_TX_FULL = "SystemStats/CANBus/TxFullCount"
LOOP_TIME = "RealOutputs/LoggedRobot/FullCycleMS"
CONSOLE = "RealOutputs/Console"

# ---- Report thresholds ---------------------------------------------------------------------------
TEMP_WARN_C = 70.0
TEMP_CRIT_C = 90.0
BATTERY_WARN_V = 9.0
BATTERY_CRIT_V = 7.5
LOOP_OVERRUN_MS = 25.0

# CTRE's generated swerve lists modules in this order, and SwerveIO logs arrays in that order.
SWERVE_MODULES = ["front left", "front right", "back left", "back right"]

_UNITS = {"Degrees": ("position", "deg"), "Rpm": ("velocity", "RPM"), "Volts": ("voltage", "V")}
_DEFAULT_TOL = {"position": 2.0, "velocity": 100.0, "voltage": 1.0}


# ================================================================================ Constants.java


def _strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    return re.sub(r"//[^\n]*", "", text)


def _block(text: str, open_idx: int) -> str:
    """Text between the brace at open_idx and its matching close brace."""
    depth = 0
    for i in range(open_idx, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[open_idx + 1 : i]
    return text[open_idx + 1 :]


def _call_args(text: str, method: str) -> list[str]:
    """Argument text of every .method(...) call in text, with nested parentheses handled."""
    out = []
    for m in re.finditer(r"\." + method + r"\s*\(", text):
        depth, start = 1, m.end()
        for i in range(start, len(text)):
            if text[i] == "(":
                depth += 1
            elif text[i] == ")":
                depth -= 1
                if depth == 0:
                    out.append(text[start:i].strip())
                    break
    return out


_FUNCS = {
    "_deg2rot": lambda x: x / 360.0,
    "_rot2deg": lambda x: x * 360.0,
    "_deg2rad": math.radians,
    "_rad2deg": math.degrees,
}


def _eval(expr: str, scope: dict):
    """Evaluates a Java arithmetic expression using constants in scope. None if it can't."""
    e = expr.strip()
    e = re.sub(r"\bUnits\.degreesToRotations\(", "_deg2rot(", e)
    e = re.sub(r"\bUnits\.rotationsToDegrees\(", "_rot2deg(", e)
    e = re.sub(r"\bUnits\.degreesToRadians\(", "_deg2rad(", e)
    e = re.sub(r"\bUnits\.radiansToDegrees\(", "_rad2deg(", e)
    e = e.replace("Math.PI", "3.141592653589793")
    e = re.sub(r"\b(\w+Constants)\.(\w+)", lambda m: m.group(2), e)  # IntakeConstants.X -> X
    e = re.sub(r"(\d)[dDfFlL]\b", r"\1", e)  # Java literal suffixes
    try:
        tree = ast.parse(e, mode="eval")
    except SyntaxError:
        return None

    def ev(node):
        if isinstance(node, ast.Expression):
            return ev(node.body)
        if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
            return float(node.value)
        if isinstance(node, ast.Name):
            v = scope.get(node.id)
            if v is None:
                raise ValueError(node.id)
            return v
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.USub, ast.UAdd)):
            v = ev(node.operand)
            return -v if isinstance(node.op, ast.USub) else v
        if isinstance(node, ast.BinOp) and isinstance(node.op, (ast.Add, ast.Sub, ast.Mult, ast.Div)):
            a, b = ev(node.left), ev(node.right)
            return {ast.Add: a + b, ast.Sub: a - b, ast.Mult: a * b,
                    ast.Div: a / b if b else math.nan}[type(node.op)]
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id in _FUNCS:
            return _FUNCS[node.func.id](*[ev(a) for a in node.args])
        raise ValueError(ast.dump(node))

    try:
        return ev(tree)
    except (ValueError, TypeError, ZeroDivisionError):
        return None


def parse_constants(path: Path) -> dict:
    """{ "IntakeConstants": {"values": {NAME: float}, "configs": {NAME: {...}}}, ... }"""
    text = _strip_comments(path.read_text(encoding="utf-8"))
    classes = {}
    raw_values: dict[str, dict[str, str]] = {}
    for m in re.finditer(r"static\s+final\s+class\s+(\w+)\s*\{", text):
        body = _block(text, m.end() - 1)
        raw_values[m.group(1)] = {
            c.group(1): c.group(2)
            for c in re.finditer(r"static\s+final\s+(?:double|int|float|long)\s+(\w+)\s*=\s*([^;]+);", body)
        }
        classes[m.group(1)] = {"body": body}
    # Constants may refer to each other (and across classes), so resolve until nothing changes.
    scope: dict[str, float] = {}
    for _ in range(5):
        changed = False
        for values in raw_values.values():
            for name, expr in values.items():
                if name not in scope:
                    v = _eval(expr, scope)
                    if v is not None:
                        scope[name] = v
                        changed = True
        if not changed:
            break
    for cname, info in classes.items():
        body = info.pop("body")
        info["values"] = {n: scope[n] for n in raw_values[cname] if n in scope}
        configs = {}
        for c in re.finditer(r"static\s+final\s+TalonFXConfiguration\s+(\w+)\s*=", body):
            stmt = body[c.end(): body.find(";", c.end())]
            def arg(method):
                found = _call_args(stmt, method)
                return _eval(found[0], scope) if found else None
            def enabled(method):
                found = _call_args(stmt, method)
                return bool(found) and found[0] == "true"
            configs[c.group(1)] = {
                "ratio": arg("withSensorToMechanismRatio"),
                "stator_limit": arg("withStatorCurrentLimit") if enabled("withStatorCurrentLimitEnable") else None,
                "supply_limit": arg("withSupplyCurrentLimit") if enabled("withSupplyCurrentLimitEnable") else None,
                "forward_soft_rot": arg("withForwardSoftLimitThreshold") if enabled("withForwardSoftLimitEnable") else None,
                "reverse_soft_rot": arg("withReverseSoftLimitThreshold") if enabled("withReverseSoftLimitEnable") else None,
            }
        info["configs"] = configs
    return classes


def find_constants_file(repo: Path = REPO_ROOT) -> Path | None:
    preferred = repo / SRC / "Constants.java"
    if preferred.is_file():
        return preferred
    found = list((repo / "src").rglob("Constants.java")) if (repo / "src").is_dir() else []
    return found[0] if found else None


# ================================================================================ PD channel names


def parse_pd_channels(repo: Path = REPO_ROOT) -> dict[int, str]:
    """Reads the '| PDH channel | Device |' table in docs/ROBOT_OVERVIEW.md."""
    doc = repo / "docs" / "ROBOT_OVERVIEW.md"
    if not doc.is_file():
        return {}
    out, in_table = {}, False
    for line in doc.read_text(encoding="utf-8").splitlines():
        cells = [c.strip() for c in line.strip().strip("|").split("|")] if line.strip().startswith("|") else None
        if cells is None:
            in_table = False
            continue
        if len(cells) >= 2 and "channel" in cells[0].lower() and "device" in cells[1].lower():
            in_table = True
            continue
        if in_table and cells[0].isdigit() and cells[1] and not set(cells[1]) <= {"-", " "}:
            out[int(cells[0])] = cells[1]
    return out


# ================================================================================ the model


class RobotModel:
    """Mechanisms, swerve, shooting, and camera info for one log, plus notes on any defaults used."""

    def __init__(self, keys: list[str], type_of, repo: Path = REPO_ROOT):
        self.notes: list[str] = []
        cpath = find_constants_file(repo)
        self.constants = parse_constants(cpath) if cpath else {}
        if not cpath:
            self.notes.append("Constants.java not found; tolerances and limits use defaults.")
        outs = [k.split("Outputs/", 1)[1] for k in keys if k.split("/", 1)[0] in ("RealOutputs", "ReplayOutputs")]
        out_set = set(outs)
        key_set = set(keys)

        self.mechanisms: dict[str, dict] = {}
        for k in sorted(out_set):
            m = re.fullmatch(r"(\w+)/(\w+)/Setpoint(Degrees|Rpm|Volts)", k)
            if not m or m.group(1) == "StateHandler":
                continue
            sub, mech, unit = m.groups()
            self.mechanisms[self._unique_name(mech, sub)] = self._mechanism(sub, mech, unit, key_set, type_of, repo)

        swerve_const = self.constants.get("SwerveConstants", {}).get("values", {})
        heading_tol = next((v for n, v in swerve_const.items() if "HEADING_TOLERANCE" in n), None)
        if heading_tol is None:
            heading_tol = 5.0
            self.notes.append("No HEADING_TOLERANCE in SwerveConstants; aiming checks use 5 deg.")
        self.swerve = {
            "module_names": SWERVE_MODULES,
            "drive_stator": "Swerve/DriveStatorCurrentAmps",
            "drive_temp": "Swerve/DriveTemperatureCelsius",
            "drive_connected": "Swerve/DriveConnected",
            "steer_connected": "Swerve/SteerConnected",
            "encoder_connected": "Swerve/EncoderConnected",
            "gyro_connected": "Swerve/GyroConnected",
            "failed_daqs": "Swerve/OdometryFailedDaqs",
            "heading_error": "RealOutputs/Swerve/FacingHeading/ErrorDegrees",
            "state": "RealOutputs/StateHandler/Swerve/State",
            "heading_tolerance": heading_tol,
        }
        self.cameras = sorted({k.split("/")[1] for k in keys
                               if k.startswith("Vision/") and k.endswith("/Connected") and k.count("/") == 2})

        ready = next((k for k in outs if k.endswith("/ReadyToShoot")), None)
        shoot_sub = ready.split("/")[0] if ready else None
        sub_mechs = [m for m in self.mechanisms.values() if m["subsystem"] == shoot_sub]
        self.shooting = {
            "ready": f"RealOutputs/{ready}" if ready else None,
            # Commands named like shooting actions (ShootGamePieces, FerryGamePieces, ...).
            "shoot_commands": [f"RealOutputs/{k}" for k in outs if k.startswith("Commands/")
                               and re.search(r"shoot|ferry", k.split("/", 1)[1], re.I)],
            "homed": [f"RealOutputs/{k}" for k in outs if k.endswith("/Homed")],
            "position_mech": next((m for m in sub_mechs if m["kind"] == "position"), None),
            "velocity_mech": next((m for m in sub_mechs if m["kind"] == "velocity"), None),
        }
        self.pd_channels = parse_pd_channels(repo)

    def _unique_name(self, mech: str, sub: str) -> str:
        name = mech.lower()
        return name if name not in self.mechanisms else f"{sub.lower()}_{name}"

    def _mechanism(self, sub, mech, unit, key_set, type_of, repo) -> dict:
        kind, units = _UNITS[unit]
        cls = self.constants.get(f"{sub}Constants", {})
        values, configs = cls.get("values", {}), cls.get("configs", {})
        cname = f"{mech.upper()}_CONFIG"
        if cname not in configs and len(configs) == 1:
            cname = next(iter(configs))
        cfg = configs.get(cname, {})
        tols = [n for n in values if "TOLERANCE" in n]
        tol_name = next((n for n in tols if mech.upper() in n), tols[0] if len(tols) == 1 else None)
        label = f"{sub} {mech.lower()}"
        if tol_name is None:
            self.notes.append(f"{label}: no tolerance found in {sub}Constants; using {_DEFAULT_TOL[kind]} {units}.")
        if not cfg:
            self.notes.append(f"{label}: no TalonFXConfiguration found in {sub}Constants; limit checks skipped.")

        measured = {"position": f"{sub}/{mech}PositionDegrees", "velocity": f"{sub}/{mech}VelocityRpm",
                    "voltage": f"{sub}/{mech}AppliedVolts"}[kind]
        stator = f"{sub}/{mech}StatorCurrentAmps"
        multi = (type_of(stator) or "").endswith("[]")
        soft = None
        if kind == "position" and cfg.get("forward_soft_rot") is not None and cfg.get("reverse_soft_rot") is not None:
            soft = (cfg["reverse_soft_rot"] * 360.0, cfg["forward_soft_rot"] * 360.0)
        files = [f"{SRC}/subsystems/{sub.lower()}/{sub}Subsystem.java",
                 f"{SRC}/subsystems/{sub.lower()}/{sub}IO.java",
                 f"{SRC}/statemachines/{sub}StateMachine.java"]
        files = [p for p in files if (repo / p).is_file()]
        files.append(f"{SRC}/Constants.java ({sub}Constants" + (f".{cname})" if cfg else ")"))
        ratio = cfg.get("ratio")
        # Multi-motor mechanisms name their motors in the subsystem, e.g.
        # DRUM_MOTOR_NAMES = {"leader", "follower one", ...}; same order as the logged arrays.
        motor_names = []
        sub_file = repo / SRC / "subsystems" / sub.lower() / f"{sub}Subsystem.java"
        if multi and sub_file.is_file():
            m = re.search(mech.upper() + r'_MOTOR_NAMES\s*=\s*\{([^}]*)\}', sub_file.read_text(encoding="utf-8"))
            if m:
                motor_names = re.findall(r'"([^"]*)"', m.group(1))
        return {
            "motor_names": motor_names,
            "label": label,
            "subsystem": sub,
            "kind": kind,
            "units": units,
            "measured": measured,
            "velocity": f"{sub}/{mech}VelocityDegreesPerSecond" if kind == "position" else None,
            "setpoint": f"RealOutputs/{sub}/{mech}/Setpoint{unit}",
            "applied_state": f"RealOutputs/{sub}/{mech}/AppliedState",
            "desired_state": f"RealOutputs/StateHandler/{sub}/Desired{mech}State",
            "current_state": f"RealOutputs/StateHandler/{sub}/Current{mech}State",
            "volts": f"{sub}/{mech}AppliedVolts",
            "stator": stator,
            "supply": f"{sub}/{mech}SupplyCurrentAmps",
            "temp": f"{sub}/{mech}TemperatureCelsius",
            "connected": f"{sub}/{mech}Connected",
            "multi_motor": multi,
            "tolerance": values[tol_name] if tol_name else _DEFAULT_TOL[kind],
            "stator_limit": cfg.get("stator_limit"),
            "supply_limit": cfg.get("supply_limit"),
            "soft_limits": soft,
            "motors": f"gear ratio {ratio:g}:1" if ratio else "gear ratio unknown",
            "files": files,
        }

    def motor_name(self, mech: dict, index: int) -> str:
        names = mech.get("motor_names") or []
        if index < len(names):
            return names[index]
        return "leader" if index == 0 else f"follower {index}"
