"""Thin transport over evocode's HSFZ/UDS — the only car-touching layer.

One socket to the FEM gateway (0x10) reaches every module by the UDS target byte
(no routing-activation; these cars are HSFZ, not DoIP). Everything here is read-only
except `clear_dtcs`, which the caller must opt into.

Phase 0 connects from a dev machine over the ENET cable. Phase 1's native collector
will speak the same wire to the gateway from the head unit itself (no adapter).
"""
from __future__ import annotations

import sys
from dataclasses import dataclass
from typing import List, Optional, Tuple

from .signals import OBD, DYN, DYN_DID, Signal

# evocode provides the transport. Prefer the installed package; fall back to the dev checkout.
try:
    from evocode.hsfz import Hsfz
except Exception:  # pragma: no cover - dev convenience: fall back to a local evocode checkout
    import os
    _ev = os.environ.get("EVOCODE_PATH") or os.path.expanduser("~/Documents/evocode")
    if os.path.isdir(_ev):
        sys.path.insert(0, _ev)
    from evocode.hsfz import Hsfz  # noqa: E402

FUNCTIONAL = 0xDF  # functional/broadcast target
TESTER_PRESENT = b"\x3e\x00"

# Best-effort F30 module map for labelling + a fallback scan if functional discovery is quiet.
KNOWN_MODULES = {
    0x10: "FEM/BDC gateway",
    0x12: "DDE (engine, N57)",
    0x18: "EGS (transmission)",
    0x63: "NBT EVO head unit",
    0x29: "DSC (brakes/stability)",
    0x40: "EPS (steering)",
    0x72: "ACSM (airbag)",
    0x78: "IHKA (climate)",
}


class DiagError(Exception):
    pass


@dataclass
class Reading:
    sig: Signal
    raw: Optional[bytes]
    value: Optional[float]
    ok: bool
    err: str = ""


def connect(gateway_ip: str, timeout: float = 15.0) -> Hsfz:
    return Hsfz(gateway_ip, timeout=timeout)


def raw(hsfz: Hsfz, ecu: int, uds: bytes, p2_star: float = 6.0) -> bytes:
    """Send a UDS request to one ECU; return the full response, raising on NRC/transport."""
    try:
        resp = hsfz.request(ecu, uds, p2_star=p2_star)
    except TypeError:
        resp = hsfz.request(ecu, uds)  # older signature
    if not resp:
        raise DiagError("empty response")
    if resp[0] == 0x7F:
        nrc = resp[2] if len(resp) > 2 else 0
        raise DiagError("NRC 0x%02X" % nrc)
    return bytes(resp)


def obd(hsfz: Hsfz, ecu: int, pid: int) -> bytes:
    """Standard mode-01: return the data bytes after `41 <pid>`."""
    r = raw(hsfz, ecu, bytes([0x01, pid]))
    if len(r) < 2 or r[0] != 0x41 or r[1] != pid:
        raise DiagError("unexpected OBD reply: " + r.hex())
    return r[2:]


def did(hsfz: Hsfz, ecu: int, did_: int) -> bytes:
    """UDS 0x22: return the data bytes after `62 <hi> <lo>`."""
    hi, lo = (did_ >> 8) & 0xFF, did_ & 0xFF
    r = raw(hsfz, ecu, bytes([0x22, hi, lo]))
    if len(r) < 3 or r[0] != 0x62 or r[1] != hi or r[2] != lo:
        raise DiagError("unexpected DID reply: " + r.hex())
    return r[3:]


def define_dyn(hsfz: Hsfz, ecu: int, source_did: int, pos: int, length: int, dyn: int = DYN_DID) -> None:
    """UDS 2C 02: (re)define dynamic DID `dyn` as source_did[pos:pos+length]. Raises on NRC.

    N57 DDE live values reject a direct 22 (NRC 31) but answer once defined into this slot,
    exactly as ISTA does. `pos` is 1-based.
    """
    sh, sl = (source_did >> 8) & 0xFF, source_did & 0xFF
    dh, dl = (dyn >> 8) & 0xFF, dyn & 0xFF
    r = raw(hsfz, ecu, bytes([0x2C, 0x02, dh, dl, sh, sl, pos & 0xFF, length & 0xFF]))
    if len(r) < 4 or r[0] != 0x6C or r[1] != 0x02 or r[2] != dh or r[3] != dl:
        raise DiagError("unexpected 2C reply: " + r.hex())


def dyn_did(hsfz: Hsfz, ecu: int, source_did: int, length: int, pos: int = 1) -> bytes:
    """Define F303 from source_did (pos,length), then read it; return the data bytes.

    ponytail: re-defines the slot on every read (idempotent for one reader). F303 is shared
    on the DDE, so a second concurrent reader could clobber it — out of our single-laptop scope.
    """
    define_dyn(hsfz, ecu, source_did, pos, length)
    return did(hsfz, ecu, DYN_DID)


def read_signal(hsfz: Hsfz, sig: Signal) -> Reading:
    """Read + decode one signal, gating the value through its plausibility bounds."""
    if sig.decode is None or sig.addr == 0:
        return Reading(sig, None, None, False, "no address/decoder yet (probe target)")
    try:
        if sig.service == OBD:
            data = obd(hsfz, sig.ecu, sig.addr)
        elif sig.service == DYN:
            data = dyn_did(hsfz, sig.ecu, sig.addr, sig.decode.width)
        else:
            data = did(hsfz, sig.ecu, sig.addr)
    except DiagError as e:
        return Reading(sig, None, None, False, str(e))
    try:
        val = sig.decode(data)
    except Exception as e:  # noqa: BLE001 - a short reply shouldn't crash a sweep
        return Reading(sig, data, None, False, "decode: %s" % e)
    if not sig.plausible(val):
        return Reading(sig, data, val, False, "out of range (unverified decode?)")
    return Reading(sig, data, val, True)


def discover_modules(hsfz: Hsfz, settle: float = 1.5) -> List[Tuple[int, str]]:
    """Enumerate awake ECUs. Functional TesterPresent first; fall back to probing KNOWN_MODULES.

    The functional path (one `3E 00` to 0xDF, every awake ECU answers `7E` from its own
    address) depends on the transport exposing per-frame source addresses; if it doesn't,
    we probe the known addresses individually.
    """
    found: dict[int, str] = {}
    try:
        hsfz.send_only(FUNCTIONAL, TESTER_PRESENT)
        while True:
            src, payload = hsfz.recv_frame(timeout=settle)
            if payload and payload[0] == 0x7E:
                found.setdefault(src, KNOWN_MODULES.get(src, "?"))
    except Exception:
        pass  # timeout / unsupported -> fall through to the scan
    if not found:
        for addr, name in KNOWN_MODULES.items():
            try:
                raw(hsfz, addr, TESTER_PRESENT, p2_star=1.0)
                found[addr] = name
            except Exception:
                continue
    # try to label unknowns by their ECU-name DID (F197), best-effort
    for addr in list(found):
        if found[addr] in ("?", ""):
            try:
                name = did(hsfz, addr, 0xF197).decode("latin1").strip("\x00 ")
                if name:
                    found[addr] = name
            except Exception:
                pass
    return sorted(found.items())
