"""Python side of the simulator <-> autopilot protocol.

Mirrors core/include/autopilot/protocol.h exactly: packed little-endian
structs, CRC-16/CCITT-FALSE appended, COBS-encoded, 0x00-terminated.
"""

import struct
from dataclasses import dataclass, field

SENSORS = ord("S")
COMMAND = ord("C")
ACTUATORS = ord("A")
MAX_PAYLOAD = 64
MAX_FRAME = 69

_SENSORS_FMT = "<BBHI3f3ff"  # 36 bytes
_COMMAND_FMT = "<BBHff"  # 12 bytes
_ACTUATORS_FMT = "<BBH3f4f3f3fI"  # 60 bytes

assert struct.calcsize(_SENSORS_FMT) == 36
assert struct.calcsize(_COMMAND_FMT) == 12
assert struct.calcsize(_ACTUATORS_FMT) == 60


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0])
    code_idx, code = 0, 1
    for byte in data:
        if byte == 0:
            out[code_idx] = code
            code_idx, code = len(out), 1
            out.append(0)
        else:
            out.append(byte)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx, code = len(out), 1
                out.append(0)
    out[code_idx] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        if code == 0:
            raise ValueError("zero byte inside COBS frame")
        i += 1
        block = data[i : i + code - 1]
        if len(block) != code - 1 or 0 in block:
            raise ValueError("truncated or corrupt COBS block")
        out += block
        i += code - 1
        if code != 0xFF and i != len(data):
            out.append(0)
    return bytes(out)


def make_frame(payload: bytes) -> bytes:
    if not 1 <= len(payload) <= MAX_PAYLOAD:
        raise ValueError("payload must contain 1..64 bytes")
    crc = crc16_ccitt(payload)
    return cobs_encode(payload + struct.pack("<H", crc)) + b"\x00"


class FrameReceiver:
    """Feed it bytes; it yields complete, CRC-checked payloads."""

    def __init__(self):
        self._buf = bytearray()
        self._overflow = False
        self.crc_errors = 0
        self.framing_errors = 0

    def feed(self, data: bytes):
        for byte in data:
            if byte != 0:
                if len(self._buf) < MAX_FRAME:
                    self._buf.append(byte)
                else:
                    self._overflow = True
                continue
            raw, self._buf = bytes(self._buf), bytearray()
            if self._overflow:
                self._overflow = False
                self.framing_errors += 1
                continue
            if not raw:
                continue
            try:
                decoded = cobs_decode(raw)
            except ValueError:
                self.framing_errors += 1
                continue
            if len(decoded) < 3 or len(decoded) > MAX_PAYLOAD + 2:
                self.framing_errors += 1
                continue
            payload, crc = decoded[:-2], struct.unpack("<H", decoded[-2:])[0]
            if crc16_ccitt(payload) != crc:
                self.crc_errors += 1
                continue
            yield payload


@dataclass
class Sensors:
    seq: int
    t_us: int
    gyro: tuple
    accel: tuple
    depth: float

    def pack(self) -> bytes:
        return struct.pack(_SENSORS_FMT, SENSORS, 0, self.seq & 0xFFFF, self.t_us & 0xFFFFFFFF,
                           *self.gyro, *self.accel, self.depth)


@dataclass
class Command:
    enable: bool
    depth_setpoint: float
    prop_rpm: float

    def pack(self) -> bytes:
        return struct.pack(_COMMAND_FMT, COMMAND, 1 if self.enable else 0, 0, self.depth_setpoint, self.prop_rpm)


@dataclass
class Actuators:
    enabled: int
    seq: int
    stern_plane: float
    rudder: float
    prop_rpm: float
    roll: float
    pitch: float
    bias_p: float
    bias_q: float
    p_roll: float
    p_pitch: float
    p_roll_pitch: float
    depth_cmd: float
    pitch_cmd: float
    depth_rate: float
    compute_cycles: int = field(default=0)

    @classmethod
    def unpack(cls, payload: bytes) -> "Actuators":
        v = struct.unpack(_ACTUATORS_FMT, payload)
        if v[0] != ACTUATORS:
            raise ValueError("not an Actuators message")
        return cls(*v[1:])
