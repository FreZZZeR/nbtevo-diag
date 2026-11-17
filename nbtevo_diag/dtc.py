"""Fault memory (item 0): read and clear DTCs per module.

`19 02 FF` (reportDTCByStatusMask, all bits) in the default session, no SecurityAccess —
safe on every module. Clearing (`14 FF FF FF`) is a write and is gated behind commit=True.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List

from . import transport

# ISO 14229 DTC status bits
STATUS_BITS = [
    (0x01, "testFailed"),
    (0x02, "failedThisCycle"),
    (0x04, "pending"),
    (0x08, "confirmed"),
    (0x10, "notCompletedSinceClear"),
    (0x20, "failedSinceClear"),
    (0x40, "notCompletedThisCycle"),
    (0x80, "warningIndicator"),
]


@dataclass
class Dtc:
    code: int          # 3-byte DTC
    status: int        # 1-byte status mask

    @property
    def hex(self) -> str:
        return "%06X" % self.code

    def flags(self) -> List[str]:
        return [name for bit, name in STATUS_BITS if self.status & bit]

    @property
    def active(self) -> bool:
        return bool(self.status & (0x01 | 0x08))  # testFailed or confirmed

    def __str__(self) -> str:
        return "%s  [%02X %s]" % (self.hex, self.status, ",".join(self.flags()) or "-")


def read_dtcs(hsfz, ecu: int) -> List[Dtc]:
    """reportDTCByStatusMask(0xFF). Returns every stored DTC with its status byte."""
    r = transport.raw(hsfz, ecu, b"\x19\x02\xff")
    if len(r) < 3 or r[0] != 0x59 or r[1] != 0x02:
        raise transport.DiagError("unexpected DTC reply: " + r.hex())
    body = r[3:]  # after 59 02 <statusAvailabilityMask>
    out = []
    for i in range(0, len(body) - 3, 4):
        code = (body[i] << 16) | (body[i + 1] << 8) | body[i + 2]
        out.append(Dtc(code, body[i + 3]))
    return out


def clear_dtcs(hsfz, ecu: int, commit: bool = False) -> bool:
    """clearDiagnosticInformation(0xFFFFFF). A WRITE — no-op unless commit=True."""
    if not commit:
        return False
    r = transport.raw(hsfz, ecu, b"\x14\xff\xff\xff")
    if not r or r[0] != 0x54:
        raise transport.DiagError("clear rejected: " + r.hex())
    return True


def scan_all(hsfz, modules) -> dict:
    """Read DTCs from each (addr, name) module; returns {addr: [Dtc]|exception-string}."""
    result = {}
    for addr, _name in modules:
        try:
            result[addr] = read_dtcs(hsfz, addr)
        except Exception as e:  # noqa: BLE001
            result[addr] = str(e)
    return result
