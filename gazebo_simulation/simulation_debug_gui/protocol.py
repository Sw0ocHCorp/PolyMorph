"""
Decoding of the robomorph protobuf messages that arrive over UDP.

Why a hand-rolled decoder?
--------------------------
The messages are robomorph's own prost-generated types. We don't have the
.proto files here, so this module reads the protobuf *wire format* directly and
maps field numbers -> names using the schemas below. The wire format is simple
and stable; the only thing you may need to tweak is the field NUMBERS / integer
types if robomorph's #[prost(...)] attributes don't follow the conventional
"sequential, starting at 1, in declaration order" layout.

If you'd rather use the real .proto files: generate Python classes with
`protoc --python_out=. robomorph.proto` and swap decode_* to use them. The rest
of the app only depends on the dict shapes returned here.

Wire types (protobuf):
    0 varint   (int32/64, uint, bool, enum, sint*)
    1 64-bit   (double, fixed64, sfixed64)
    2 length-delimited (bytes, string, sub-message, packed repeated)
    5 32-bit   (float, fixed32, sfixed32)
"""
from __future__ import annotations

import struct
from dataclasses import dataclass, field
from typing import Any

import config


# ---------------------------------------------------------------------------
# Low-level wire-format reader
# ---------------------------------------------------------------------------
def _read_varint(buf: bytes, i: int) -> tuple[int, int]:
    result = 0
    shift = 0
    while True:
        b = buf[i]
        i += 1
        result |= (b & 0x7F) << shift
        if not (b & 0x80):
            return result, i
        shift += 7
        if shift > 70:
            raise ValueError("varint too long")


def parse_fields(buf: bytes) -> dict[int, list[tuple[int, Any]]]:
    """
    Generic parse: returns {field_number: [(wire_type, raw_value), ...]}.
    Raw value is:
       wire 0 -> int (varint)
       wire 1 -> bytes (8)         (caller interprets as double/fixed64)
       wire 2 -> bytes (payload)   (sub-message / string / packed)
       wire 5 -> bytes (4)
    Repeated fields keep every occurrence. Raises on malformed input so the
    classifier can reject junk datagrams.
    """
    out: dict[int, list[tuple[int, Any]]] = {}
    i, n = 0, len(buf)
    while i < n:
        tag, i = _read_varint(buf, i)
        field_no = tag >> 3
        wire = tag & 0x07
        if field_no == 0:
            raise ValueError("field number 0")
        if wire == 0:
            val, i = _read_varint(buf, i)
        elif wire == 1:
            val = buf[i:i + 8]
            if len(val) != 8:
                raise ValueError("truncated 64-bit")
            i += 8
        elif wire == 2:
            ln, i = _read_varint(buf, i)
            val = buf[i:i + ln]
            if len(val) != ln:
                raise ValueError("truncated length-delimited")
            i += ln
        elif wire == 5:
            val = buf[i:i + 4]
            if len(val) != 4:
                raise ValueError("truncated 32-bit")
            i += 4
        else:
            raise ValueError(f"unsupported wire type {wire}")
        out.setdefault(field_no, []).append((wire, val))
    return out


def _f64(raw: bytes) -> float:
    return struct.unpack("<d", raw)[0]


def _first(fields: dict[int, list], no: int, default=0):
    """Return the raw value of the first occurrence of `no`, or default."""
    lst = fields.get(no)
    return lst[0][1] if lst else default


def _dbl(fields: dict[int, list], no: int) -> float:
    lst = fields.get(no)
    if not lst:
        return 0.0                      # proto3 omits zero-valued scalars
    wire, raw = lst[0]
    return _f64(raw) if wire == 1 else float(raw)


def _int(fields: dict[int, list], no: int) -> int:
    lst = fields.get(no)
    if not lst:
        return 0
    wire, raw = lst[0]
    if wire == 0:
        return int(raw)
    if wire == 1:
        return struct.unpack("<q", raw)[0]
    return 0


# ---------------------------------------------------------------------------
# robomorph message schemas  (field-number -> meaning)
# ---------------------------------------------------------------------------
# Adjust these numbers/types if your prost attributes differ. Defaults follow
# the struct declaration order in main.rs / robomorph.

@dataclass
class Gnss:
    latitude: float = 0.0    # 1
    longitude: float = 0.0   # 2
    altitude: float = 0.0    # 3
    fix_status: int = 0      # 4


@dataclass
class Imu:
    l_accel_x: float = 0.0   # 1
    l_accel_y: float = 0.0   # 2
    l_accel_z: float = 0.0   # 3
    a_velocity_x: float = 0.0  # 4
    a_velocity_y: float = 0.0  # 5
    a_velocity_z: float = 0.0  # 6
    magnetic_field_x: float = 0.0  # 7
    magnetic_field_y: float = 0.0  # 8
    magnetic_field_z: float = 0.0  # 9


@dataclass
class Motor:
    id: int = 0                  # 1
    status: int = 0              # 2  (1=IDLE, 2=RUNNING, 3=ERROR)
    current_value: float = 0.0   # 3
    setpoint_value: float = 0.0  # 4
    command_type: int = 0        # 5  (5=ANGULARPOSITION, 6=VELOCITY, 7=TORQUE)
    min_value: float = 0.0       # 6
    max_value: float = 0.0       # 7
    p: float = 0.0               # 8
    i: float = 0.0               # 9
    d: float = 0.0               # 10


@dataclass
class Ray:
    vertical_angle: float = 0.0    # 1
    horizontal_angle: float = 0.0  # 2
    length: float = 0.0            # 3


@dataclass
class Lidar:
    vertical_min_angle: float = 0.0          # 1
    vertical_angle_resolution: float = 0.0   # 2
    vertical_max_angle: float = 0.0          # 3
    horizontal_min_angle: float = 0.0        # 4
    horizontal_angle_resolution: float = 0.0  # 5
    horizontal_max_angle: float = 0.0        # 6
    rays: list[Ray] = field(default_factory=list)  # 7 (repeated message)


# Human-readable enum labels (match robomorph motor_messages).
COMMAND_TYPE = {5: "ANGULARPOSITION", 6: "VELOCITY", 7: "TORQUE"}
MOTOR_STATUS = {1: "IDLE", 2: "RUNNING", 3: "ERROR"}


# ---------------------------------------------------------------------------
# Typed decoders
# ---------------------------------------------------------------------------
def decode_gnss(buf: bytes) -> Gnss:
    f = parse_fields(buf)
    return Gnss(_dbl(f, 1), _dbl(f, 2), _dbl(f, 3), _int(f, 4))


def decode_imu(buf: bytes) -> Imu:
    f = parse_fields(buf)
    return Imu(_dbl(f, 1), _dbl(f, 2), _dbl(f, 3),
               _dbl(f, 4), _dbl(f, 5), _dbl(f, 6),
               _dbl(f, 7), _dbl(f, 8), _dbl(f, 9))


def decode_motor(buf: bytes) -> Motor:
    # new MotorFeedBack tags:
    # 1 id · 2 status · 3 current · 4 setpoint · 5 command_type · 6 min · 7 max · 8 p · 9 i · 10 d
    f = parse_fields(buf)
    return Motor(
        id=_int(f, 1),
        status=_int(f, 2),
        current_value=_dbl(f, 3),
        setpoint_value=_dbl(f, 4),
        command_type=_int(f, 5),
        min_value=_dbl(f, 6),
        max_value=_dbl(f, 7),
        p=_dbl(f, 8),
        i=_dbl(f, 9),
        d=_dbl(f, 10),
    )


def _decode_ray(buf: bytes) -> Ray:
    f = parse_fields(buf)
    return Ray(_dbl(f, 1), _dbl(f, 2), _dbl(f, 3))


def decode_lidar(buf: bytes) -> Lidar:
    f = parse_fields(buf)
    rays = [_decode_ray(raw) for (wire, raw) in f.get(7, []) if wire == 2]
    return Lidar(_dbl(f, 1), _dbl(f, 2), _dbl(f, 3),
                 _dbl(f, 4), _dbl(f, 5), _dbl(f, 6), rays)


# ---------------------------------------------------------------------------
# Heuristic classifier  (used when UDP_TYPED_FRAMING is False)
# ---------------------------------------------------------------------------
def classify(buf: bytes) -> str | None:
    """
    Best-effort guess of which message a raw datagram is, using its field
    layout. Returns 'gnss' | 'imu' | 'motor' | 'lidar' | None.

    Signals:
      * field 7 present as a length-delimited (sub-message) -> lidar (rays)
      * field 1 is a varint (int id)                        -> motor
      * field 1 is a 64-bit double:
            - has any field >= 5  -> imu (9 doubles)
            - else                -> gnss (3 doubles + int fix_status)
    Prefer typed framing to avoid this entirely.
    """
    try:
        f = parse_fields(buf)
    except ValueError:
        return None
    if not f:
        return None

    if any(w == 2 for (w, _) in f.get(7, [])):
        return "lidar"

    first = f.get(1)
    if first and first[0][0] == 0:        # field 1 is a varint -> motor id
        return "motor"

    max_field = max(f)
    if max_field >= 5:
        return "imu"
    return "gnss"


def split_datagram(data: bytes) -> tuple[str | None, bytes]:
    """
    Turn one raw datagram into (kind, payload).

    Typed framing: first byte is the type id from config, rest is protobuf.
    Untyped:       run the heuristic classifier on the whole datagram.
    """
    if config.UDP_TYPED_FRAMING:
        if not data:
            return None, b""
        tid, payload = data[0], data[1:]
        return {
            config.MSG_GNSS: "gnss",
            config.MSG_IMU: "imu",
            config.MSG_MOTOR: "motor",
            config.MSG_LIDAR: "lidar",
        }.get(tid), payload
    return classify(data), data


DECODERS = {
    "gnss": decode_gnss,
    "imu": decode_imu,
    "motor": decode_motor,
    "lidar": decode_lidar,
}


# ---------------------------------------------------------------------------
# Self-test: hand-encode known messages and round-trip them.
# ---------------------------------------------------------------------------
def _encode_varint(n: int) -> bytes:
    out = bytearray()
    while True:
        b = n & 0x7F
        n >>= 7
        out.append(b | (0x80 if n else 0))
        if not n:
            return bytes(out)


def _tag(no: int, wire: int) -> bytes:
    return _encode_varint((no << 3) | wire)


def _enc_double(no: int, v: float) -> bytes:
    return _tag(no, 1) + struct.pack("<d", v)


def _enc_varint_field(no: int, v: int) -> bytes:
    return _tag(no, 0) + _encode_varint(v)


def _enc_submsg(no: int, payload: bytes) -> bytes:
    return _tag(no, 2) + _encode_varint(len(payload)) + payload


if __name__ == "__main__":
    # IMU round-trip
    imu_bytes = b"".join(_enc_double(k, v) for k, v in [
        (1, 0.1), (2, -0.2), (3, 9.81), (4, 0.01), (5, 0.0),
        (6, -0.03), (7, 12.0), (8, 3.0), (9, -45.0)])
    imu = decode_imu(imu_bytes)
    assert abs(imu.l_accel_z - 9.81) < 1e-9 and abs(imu.magnetic_field_z + 45.0) < 1e-9
    assert classify(imu_bytes) == "imu", classify(imu_bytes)

    # GNSS round-trip (proto3 drops zero fields; keep some non-zero)
    gnss_bytes = (_enc_double(1, 48.85) + _enc_double(2, 2.35)
                  + _enc_double(3, 35.0) + _enc_varint_field(4, 1))
    g = decode_gnss(gnss_bytes)
    assert abs(g.latitude - 48.85) < 1e-9 and g.fix_status == 1
    assert classify(gnss_bytes) == "gnss", classify(gnss_bytes)

    # Motor round-trip  (new layout: 1 id,2 status,3 cur,4 set,5 cmd,6 min,7 max,8 p)
    motor_bytes = (_enc_varint_field(1, 7) + _enc_varint_field(2, 1)
                   + _enc_double(3, 1.23) + _enc_double(4, 1.5)
                   + _enc_varint_field(5, 6) + _enc_double(6, -3.14)
                   + _enc_double(7, 3.14) + _enc_double(8, 1.0))
    m = decode_motor(motor_bytes)
    assert m.id == 7 and m.status == 1 and abs(m.current_value - 1.23) < 1e-9
    assert abs(m.setpoint_value - 1.5) < 1e-9 and m.command_type == 6
    assert abs(m.min_value + 3.14) < 1e-9 and abs(m.max_value - 3.14) < 1e-9
    assert COMMAND_TYPE[m.command_type] == "VELOCITY" and MOTOR_STATUS[m.status] == "IDLE"
    assert classify(motor_bytes) == "motor", classify(motor_bytes)

    # Lidar with two rays
    ray1 = _enc_double(1, 0.0) + _enc_double(2, 0.1) + _enc_double(3, 5.0)
    ray2 = _enc_double(1, 0.0) + _enc_double(2, 0.2) + _enc_double(3, 4.5)
    lidar_bytes = (_enc_double(1, -0.5) + _enc_double(4, -3.14)
                   + _enc_double(5, 0.01) + _enc_double(6, 3.14)
                   + _enc_submsg(7, ray1) + _enc_submsg(7, ray2))
    lid = decode_lidar(lidar_bytes)
    assert len(lid.rays) == 2 and abs(lid.rays[1].length - 4.5) < 1e-9
    assert classify(lidar_bytes) == "lidar", classify(lidar_bytes)

    print("protocol.py self-test: OK (all messages round-trip & classify correctly)")