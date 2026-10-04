"""Dependency-free reader for WPILib .wpilog files (the format AdvantageKit writes).

Format spec: https://github.com/wpilibsuite/allwpilib/blob/main/wpiutil/doc/datalog.adoc

Usage:
    log = WpiLog.open("akit_26-10-03_18-00-00.wpilog")
    s = log.series("Intake/ArmPositionDegrees")   # leading "/" and "AdvantageKit/" optional
    for t, v in zip(s.times, s.values): ...        # t is seconds since log start

Only the records for keys you ask for are decoded, so large match logs stay fast.
"""

from __future__ import annotations

import bisect
import json
import struct
from dataclasses import dataclass, field
from pathlib import Path

_HEADER_MAGIC = b"WPILOG"


@dataclass
class Entry:
    id: int
    name: str
    type: str
    metadata: str
    # Raw record locations: (timestamp_us, payload_start, payload_end)
    records: list = field(default_factory=list)


@dataclass
class Series:
    """A decoded time series. times are seconds since the first record in the log."""

    key: str
    type: str
    times: list
    values: list

    def __len__(self):
        return len(self.times)

    def at(self, t: float):
        """Sample-and-hold value at time t (the last value logged at or before t), or None."""
        i = bisect.bisect_right(self.times, t) - 1
        return self.values[i] if i >= 0 else None

    def between(self, start: float, end: float) -> "Series":
        """Values logged in [start, end], plus the held value at start if one exists."""
        lo = bisect.bisect_left(self.times, start)
        hi = bisect.bisect_right(self.times, end)
        times, values = self.times[lo:hi], self.values[lo:hi]
        if lo > 0 and (not times or times[0] > start):
            times = [start] + times
            values = [self.values[lo - 1]] + values
        return Series(self.key, self.type, times, values)


def normalize_key(name: str) -> str:
    """Maps every spelling of a key to one form: no leading slash, no AdvantageKit/ prefix."""
    name = name.lstrip("/")
    if name.startswith("AdvantageKit/"):
        name = name[len("AdvantageKit/") :]
    return name


class WpiLog:
    def __init__(self, data: bytes, path: str = ""):
        self.path = path
        self._data = data
        self.extra_header = ""
        self.entries: dict[str, Entry] = {}  # normalized key -> Entry
        self._by_id: dict[int, Entry] = {}
        self._structs: dict[str, list] = {}  # struct name -> parsed field list
        self.start_us: int | None = None
        self.end_us: int | None = None
        self._index()

    @classmethod
    def open(cls, path) -> "WpiLog":
        return cls(Path(path).read_bytes(), str(path))

    # ------------------------------------------------------------------ indexing

    def _index(self):
        data = self._data
        if data[:6] != _HEADER_MAGIC:
            raise ValueError(f"{self.path}: not a WPILOG file")
        version = struct.unpack_from("<H", data, 6)[0]
        if version != 0x0100:
            raise ValueError(f"{self.path}: unsupported WPILOG version {version:#06x}")
        extra_len = struct.unpack_from("<I", data, 8)[0]
        self.extra_header = data[12 : 12 + extra_len].decode("utf-8", "replace")
        pos = 12 + extra_len
        n = len(data)
        by_id = self._by_id
        from_bytes = int.from_bytes

        while pos < n:
            bits = data[pos]
            id_len = (bits & 0x3) + 1
            size_len = ((bits >> 2) & 0x3) + 1
            ts_len = ((bits >> 4) & 0x7) + 1
            hdr_end = pos + 1 + id_len + size_len + ts_len
            if hdr_end > n:
                break  # truncated final record (robot lost power mid-write)
            p = pos + 1
            entry_id = from_bytes(data[p : p + id_len], "little")
            p += id_len
            size = from_bytes(data[p : p + size_len], "little")
            p += size_len
            ts = from_bytes(data[p : p + ts_len], "little")
            start, end = hdr_end, hdr_end + size
            if end > n:
                break
            pos = end

            if self.start_us is None or ts < self.start_us:
                self.start_us = ts
            if self.end_us is None or ts > self.end_us:
                self.end_us = ts

            if entry_id == 0:
                self._control(start, end)
                continue
            entry = by_id.get(entry_id)
            if entry is not None:
                entry.records.append((ts, start, end))

        if self.start_us is None:
            self.start_us = self.end_us = 0

    def _control(self, start: int, end: int):
        data = self._data
        kind = data[start]
        if kind == 0:  # Start
            p = start + 1
            entry_id, name_len = struct.unpack_from("<II", data, p)
            p += 8
            name = data[p : p + name_len].decode("utf-8", "replace")
            p += name_len
            (type_len,) = struct.unpack_from("<I", data, p)
            p += 4
            type_ = data[p : p + type_len].decode("utf-8", "replace")
            p += type_len
            (meta_len,) = struct.unpack_from("<I", data, p)
            p += 4
            meta = data[p : p + meta_len].decode("utf-8", "replace")
            entry = Entry(entry_id, name, type_, meta)
            self._by_id[entry_id] = entry
            # If a key is restarted (rare), keep appending to the same series.
            key = normalize_key(name)
            existing = self.entries.get(key)
            if existing is not None and existing.type == type_:
                self._by_id[entry_id] = existing
            else:
                self.entries[key] = entry
        elif kind == 2:  # SetMetadata
            entry_id, meta_len = struct.unpack_from("<II", data, start + 1)
            entry = self._by_id.get(entry_id)
            if entry is not None:
                entry.metadata = data[start + 9 : start + 9 + meta_len].decode("utf-8", "replace")
        # kind == 1 (Finish) needs no handling: later records for that id are simply absent.

    # ------------------------------------------------------------------ queries

    @property
    def duration(self) -> float:
        return (self.end_us - self.start_us) / 1e6

    def keys(self, prefix: str = "") -> list[str]:
        prefix = normalize_key(prefix)
        return sorted(k for k in self.entries if k.startswith(prefix))

    def has(self, key: str) -> bool:
        return normalize_key(key) in self.entries

    def type_of(self, key: str) -> str | None:
        e = self.entries.get(normalize_key(key))
        return e.type if e else None

    def series(self, key: str) -> Series | None:
        """Decodes every value logged for key, or returns None if the key is not in the log."""
        key = normalize_key(key)
        entry = self.entries.get(key)
        if entry is None:
            return None
        decode = self._decoder(entry.type)
        t0 = self.start_us
        data = self._data
        times, values = [], []
        for ts, start, end in entry.records:
            try:
                values.append(decode(data[start:end]))
            except Exception:
                continue  # skip a corrupt record rather than failing the whole analysis
            times.append((ts - t0) / 1e6)
        return Series(key, entry.type, times, values)

    # ------------------------------------------------------------------ decoding

    def _decoder(self, type_: str):
        simple = {
            "boolean": lambda b: b[0] != 0,
            "int64": lambda b: struct.unpack("<q", b)[0],
            "float": lambda b: struct.unpack("<f", b)[0],
            "double": lambda b: struct.unpack("<d", b)[0],
            "string": lambda b: b.decode("utf-8", "replace"),
            "json": lambda b: json.loads(b.decode("utf-8", "replace") or "null"),
            "structschema": lambda b: b.decode("utf-8", "replace"),
            "boolean[]": lambda b: [x != 0 for x in b],
            "int64[]": lambda b: list(struct.unpack(f"<{len(b) // 8}q", b)),
            "float[]": lambda b: list(struct.unpack(f"<{len(b) // 4}f", b)),
            "double[]": lambda b: list(struct.unpack(f"<{len(b) // 8}d", b)),
            "string[]": _decode_string_array,
        }
        if type_ in simple:
            return simple[type_]
        if type_.startswith("struct:"):
            name = type_[len("struct:") :]
            is_array = name.endswith("[]")
            if is_array:
                name = name[:-2]

            def decode_struct(b, name=name, is_array=is_array):
                fields = self._struct_fields(name)
                if fields is None:
                    return b  # schema missing: hand back raw bytes
                if is_array:
                    size = self._struct_size(fields)
                    return [self._unpack_struct(fields, b, i * size)[0] for i in range(len(b) // size)]
                return self._unpack_struct(fields, b, 0)[0]

            return decode_struct
        return lambda b: b  # raw, proto:*, msgpack, or unknown

    def _struct_fields(self, name: str):
        if name in self._structs:
            return self._structs[name]
        schema_entry = self.entries.get(normalize_key(f".schema/struct:{name}"))
        if schema_entry is None or not schema_entry.records:
            self._structs[name] = None
            return None
        _, start, end = schema_entry.records[-1]
        schema = self._data[start:end].decode("utf-8", "replace")
        fields = []
        self._structs[name] = fields  # guard against self-reference while parsing
        for decl in schema.split(";"):
            decl = decl.strip()
            if not decl:
                continue
            if decl.startswith("enum"):
                decl = decl[decl.index("}") + 1 :].strip()
            type_name, _, fname = decl.rpartition(" ")
            type_name = type_name.strip()
            count = 1
            if "[" in fname:
                fname, _, rest = fname.partition("[")
                count = int(rest.rstrip("]"))
            if ":" in fname:  # bitfields are not used by WPILib geometry; treat as whole field
                fname = fname.split(":")[0]
            fields.append((fname, type_name, count))
        # Resolve nested struct definitions eagerly so missing schemas surface as raw bytes.
        for _, type_name, _ in fields:
            if type_name not in _PRIMITIVES and self._struct_fields(type_name) is None:
                self._structs[name] = None
                return None
        return fields

    def _struct_size(self, fields) -> int:
        total = 0
        for _, type_name, count in fields:
            if type_name in _PRIMITIVES:
                total += _PRIMITIVES[type_name][1] * count
            else:
                total += self._struct_size(self._structs[type_name]) * count
        return total

    def _unpack_struct(self, fields, b: bytes, offset: int):
        out = {}
        for fname, type_name, count in fields:
            if type_name in _PRIMITIVES:
                fmt, size = _PRIMITIVES[type_name]
                vals = struct.unpack_from(f"<{count}{fmt}", b, offset)
                offset += size * count
                if type_name == "char":
                    out[fname] = b"".join(vals).decode("utf-8", "replace").rstrip("\0")
                else:
                    out[fname] = vals[0] if count == 1 else list(vals)
            else:
                items = []
                for _ in range(count):
                    val, offset = self._unpack_struct(self._structs[type_name], b, offset)
                    items.append(val)
                out[fname] = items[0] if count == 1 else items
        return out, offset


_PRIMITIVES = {
    "bool": ("?", 1),
    "char": ("c", 1),
    "int8": ("b", 1),
    "int16": ("h", 2),
    "int32": ("i", 4),
    "int64": ("q", 8),
    "uint8": ("B", 1),
    "uint16": ("H", 2),
    "uint32": ("I", 4),
    "uint64": ("Q", 8),
    "float": ("f", 4),
    "float32": ("f", 4),
    "double": ("d", 8),
    "float64": ("d", 8),
}


def _decode_string_array(b: bytes) -> list[str]:
    (count,) = struct.unpack_from("<I", b, 0)
    p, out = 4, []
    for _ in range(count):
        (length,) = struct.unpack_from("<I", b, p)
        p += 4
        out.append(b[p : p + length].decode("utf-8", "replace"))
        p += length
    return out


def pose2d_tuple(value) -> tuple[float, float, float] | None:
    """(x_m, y_m, heading_deg) from a decoded struct:Pose2d, or None."""
    import math

    try:
        return (
            value["translation"]["x"],
            value["translation"]["y"],
            math.degrees(value["rotation"]["value"]),
        )
    except (TypeError, KeyError):
        return None
