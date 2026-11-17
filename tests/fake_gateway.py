"""A canned HSFZ/UDS gateway for offline tests — duck-types evocode's Hsfz.

Register canned replies per (ecu, request-bytes); `request()` returns the registered
response, or a UDS negative response (7F ..) for anything unregistered. This lets the
transport / DTC / signal-read paths run with no car and no real socket.

Only the one method the transport uses (`request`) is implemented — that is the whole
seam between nbtevo-diag and the wire.
"""
from __future__ import annotations


class FakeHsfz:
    def __init__(self):
        self._canned = {}  # (ecu, bytes(request)) -> bytes(response)

    # --- registration helpers -------------------------------------------------
    def on(self, ecu: int, request: bytes, response: bytes) -> "FakeHsfz":
        self._canned[(ecu, bytes(request))] = bytes(response)
        return self

    def on_dtcs(self, ecu: int, dtcs, mask: int = 0xFF) -> "FakeHsfz":
        """Register a 19 02 FF answer: `59 02 <mask>` then 4 bytes (3 code + 1 status) each."""
        body = bytearray([0x59, 0x02, mask & 0xFF])
        for code, status in dtcs:
            body += bytes([(code >> 16) & 0xFF, (code >> 8) & 0xFF, code & 0xFF, status & 0xFF])
        return self.on(ecu, b"\x19\x02\xff", bytes(body))

    def on_obd(self, ecu: int, pid: int, data: bytes) -> "FakeHsfz":
        """Register a mode-01 answer: `41 <pid>` then the data bytes."""
        return self.on(ecu, bytes([0x01, pid]), bytes([0x41, pid]) + bytes(data))

    def on_did(self, ecu: int, did: int, data: bytes) -> "FakeHsfz":
        """Register a UDS 0x22 answer: `62 <hi> <lo>` then the data bytes."""
        hi, lo = (did >> 8) & 0xFF, did & 0xFF
        return self.on(ecu, bytes([0x22, hi, lo]), bytes([0x62, hi, lo]) + bytes(data))

    # --- the one method transport.raw() calls --------------------------------
    def request(self, ecu: int, uds: bytes, p2_star: float = 6.0) -> bytes:
        resp = self._canned.get((ecu, bytes(uds)))
        if resp is None:
            # UDS negative response: 7F <sid> 0x11 (serviceNotSupported). transport.raw raises on it.
            return bytes([0x7F, uds[0] if uds else 0x00, 0x11])
        return resp
