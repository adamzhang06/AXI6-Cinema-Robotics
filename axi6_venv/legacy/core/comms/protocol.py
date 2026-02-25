"""
core/comms/protocol.py
-----------------------
Shared socket message protocol used by Mac server → Pi server.

Both sides import this module so the wire format stays in one place.

Reference: legacy/socket/mac_spline_server.py, legacy/socket/pi_motor_server.py
"""

from __future__ import annotations
import json
from enum import Enum
from dataclasses import dataclass, asdict
from typing import Any


class MessageType(str, Enum):
    # Motion commands
    MOVE          = "move"          # pan + tilt velocity
    SPLINE_START  = "spline_start"  # begin spline playback
    SPLINE_STOP   = "spline_stop"
    SPLINE_DATA   = "spline_data"   # transmit keyframe payload
    STOP          = "stop"

    # Status
    STATUS        = "status"
    ERROR         = "error"
    ACK           = "ack"


@dataclass
class Message:
    type: MessageType
    payload: dict[str, Any]

    def to_json(self) -> str:
        return json.dumps({"type": self.type.value, "payload": self.payload})

    @staticmethod
    def from_json(raw: str) -> "Message":
        data = json.loads(raw)
        return Message(
            type    = MessageType(data["type"]),
            payload = data.get("payload", {}),
        )

    # ── Convenience constructors ──────────────────────────────────────────────

    @staticmethod
    def move(pan: float, tilt: float) -> "Message":
        return Message(MessageType.MOVE, {"pan": pan, "tilt": tilt})

    @staticmethod
    def stop() -> "Message":
        return Message(MessageType.STOP, {})

    @staticmethod
    def ack(ref_type: MessageType) -> "Message":
        return Message(MessageType.ACK, {"ref": ref_type.value})

    @staticmethod
    def error(reason: str) -> "Message":
        return Message(MessageType.ERROR, {"reason": reason})
