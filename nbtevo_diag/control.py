"""xDrive control (xDelete-style temporary RWD) — an ACTUATION, with a software interlock.

Ported from ID6_apps's xDrive Control: the DSC (0x29) commands the transfer case via
`2E DB E1 <0=RWD | 1=AWD>`, after a variant check (`22 F1 50` → 0x0F2510 / 0x0F2850) and a
capability read (`22 DB E1`). Mode is read back to confirm.

THE DIFFERENCE (what you asked for): ID6_apps gates the write on **stationary, in Park, AND
engine OFF (rpm=0)** — and in BM3 form that last part is just a label the human is trusted to
honour. Here every condition is a **code-enforced fuse** — engine-off is NOT dropped, it is
turned into a software default you must consciously waive:

  * Engaging RWD requires **standstill (speed == 0)** — this floor is NOT bypassable; you never
    drop the front axle while moving.
  * Park is required by default (``require_park``), waivable with ``--no-park-check``.
  * **Engine-off is the default** (``require_engine_off``), enforced in code; waive it
    consciously with ``--allow-running`` when you mean to. It stays the default because the
    write is latch-like (the 0x0F2510 DSC only restores AWD on an ignition cycle) and the
    module tracks RWD as a degraded state — engine-off is the quiescent, zero-driveline-torque
    moment the controller expects.
  * Dry-run by default; a real write needs ``commit=True``.
  * VIN consent gate (own car), explicit, like every other write in this stack.
  * Read-back verify after the write; AWD is the safe direction and restores on an ignition cycle.

⚠ This disables front-axle drive. It changes how the car behaves (traction, DSC/ABS envelope).
Track / private land, at your own risk. Never engage it on a public road.

The DSC variant and DIDs below come from a 340i (ID6_apps). **Validate them on the 335d's DSC
before trusting a live write** — a different variant answers differently.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

from . import transport

DSC = 0x29          # stability/DSC module — commands the transfer case
EGS = 0x18          # transmission (gear / Park)
DME = 0x12          # engine (speed)

DID_VARIANT = 0xF150     # 22 F1 50 -> DSC variant index
DID_CAPABILITY = 0xDBE1  # 22 DB E1 capability; 2E DB E1 <v> is the control
DID_GEAR = 0xDA2E        # EGS: 00 = Park
SUPPORTED_VARIANTS = (0x0F2510, 0x0F2850)

RWD, AWD = 0, 1


@dataclass
class Interlock:
    ok: bool
    reasons: List[str]


def check_interlock(speed_kmh: Optional[int], in_park: Optional[bool], engine_running: Optional[bool],
                    require_park: bool = True, require_engine_off: bool = True) -> Interlock:
    """The software fuse. Pure — unit-testable without a car.

    Standstill is mandatory for engaging RWD (never bypassable). Park and engine-off are required
    by default but each waivable explicitly — engine-off is a code-enforced default, not an
    external label, and not silently dropped. An unreadable condition refuses (fail-safe).
    """
    reasons: List[str] = []
    if speed_kmh is None:
        reasons.append("could not read vehicle speed — refusing")
    elif speed_kmh != 0:
        reasons.append("vehicle is moving (%d km/h) — engage RWD only at a standstill" % speed_kmh)
    if require_park:
        if in_park is None:
            reasons.append("could not confirm Park — refusing (use require_park=False to waive)")
        elif not in_park:
            reasons.append("not in Park")
    if require_engine_off:
        if engine_running is None:
            reasons.append("could not read engine state — refusing (use --allow-running to waive)")
        elif engine_running:
            reasons.append("engine is running — switch it off, or pass --allow-running")
    return Interlock(not reasons, reasons)


def _variant(hsfz) -> Optional[int]:
    try:
        d = transport.did(hsfz, DSC, DID_VARIANT)
    except transport.DiagError:
        return None
    # first non-zero 3-byte index (as ID6_apps parses it)
    for i in range(len(d) - 2):
        idx = (d[i] << 16) | (d[i + 1] << 8) | d[i + 2]
        if idx:
            return idx
    return None


def read_speed(hsfz) -> Optional[int]:
    try:
        return transport.obd(hsfz, DME, 0x0D)[0]
    except Exception:  # noqa: BLE001
        return None


def read_park(hsfz) -> Optional[bool]:
    try:
        return transport.did(hsfz, EGS, DID_GEAR)[0] == 0
    except Exception:  # noqa: BLE001
        return None


def read_engine_running(hsfz) -> Optional[bool]:
    try:
        d = transport.obd(hsfz, DME, 0x0C)  # rpm; both bytes zero == engine off (as ID6_apps checks)
        return (d[0] | d[1]) != 0
    except Exception:  # noqa: BLE001
        return None


def read_mode(hsfz) -> Optional[str]:
    """Report the transfer case's current mode, if it answers. Best-effort (validate on car)."""
    try:
        d = transport.did(hsfz, DSC, DID_CAPABILITY)
    except transport.DiagError:
        return None
    return "AWD" if (d and d[0] == AWD) else "RWD" if (d and d[0] == RWD) else None


def status(hsfz) -> dict:
    er = read_engine_running(hsfz)
    return {
        "variant": _variant(hsfz),
        "supported": _variant(hsfz) in SUPPORTED_VARIANTS,
        "mode": read_mode(hsfz),
        "speed": read_speed(hsfz),
        "park": read_park(hsfz),
        "engine": None if er is None else ("running" if er else "off"),
    }


def set_mode(hsfz, target: int, commit: bool = False, require_park: bool = True,
             require_engine_off: bool = True) -> Tuple[bool, str]:
    """Engage RWD (target=RWD) or restore AWD (target=AWD). Dry-run unless commit=True.

    Returns (done, message). The interlock only gates *engaging RWD*; restoring AWD is the safe
    direction and is allowed (one DSC variant needs an ignition cycle instead — reported).
    """
    var = _variant(hsfz)
    if var not in SUPPORTED_VARIANTS:
        return False, "DSC variant %s not in supported set — validate on this car first" % (
            ("0x%06X" % var) if var else "unreadable")

    if target == RWD:
        lock = check_interlock(read_speed(hsfz), read_park(hsfz), read_engine_running(hsfz),
                               require_park, require_engine_off)
        if not lock.ok:
            return False, "INTERLOCK: " + "; ".join(lock.reasons)
    else:
        if var != 0x0F2850:
            return False, "this DSC restores AWD on an ignition cycle (no live write for 0x%06X)" % var

    plan = "2E DBE1 %02X  (%s) on DSC 0x29" % (target, "RWD" if target == RWD else "AWD")
    if not commit:
        return False, "DRY RUN — would send %s. Add --commit to perform it." % plan

    # NB: caller is expected to have enforced the VIN consent gate before commit.
    try:
        transport.raw(hsfz, DSC, bytes([0x2E, (DID_CAPABILITY >> 8) & 0xFF, DID_CAPABILITY & 0xFF, target]))
    except transport.DiagError as e:
        return False, "DSC refused the write (%s)" % e
    now = read_mode(hsfz)
    ok = now == ("RWD" if target == RWD else "AWD")
    return ok, ("confirmed %s by read-back" % now) if ok else "write accepted; mode unconfirmed (read-back=%s)" % now


def selftest():
    """Pure fuse checks — no car. Args: (speed, in_park, engine_running, ...)."""
    assert check_interlock(0, True, False).ok                         # stopped, Park, engine off -> ok
    assert check_interlock(0, True, False, require_park=False).ok
    assert not check_interlock(5, True, False).ok                     # moving -> blocked
    assert not check_interlock(0, False, False).ok                    # not in Park -> blocked
    assert not check_interlock(None, True, False).ok                  # unknown speed -> refuse
    assert not check_interlock(0, None, False).ok                     # unknown Park -> refuse (unless waived)
    assert check_interlock(0, None, False, require_park=False).ok
    assert not check_interlock(0, True, True).ok                      # engine running -> blocked by default
    assert check_interlock(0, True, True, require_engine_off=False).ok  # ...waived consciously
    assert not check_interlock(0, True, None).ok                      # unknown engine -> refuse
    assert check_interlock(0, True, None, require_engine_off=False).ok
    return True


if __name__ == "__main__":
    selftest()
    print("xDrive interlock selftest OK")
