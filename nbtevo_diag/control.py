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

Two DSC generations are handled: the 340i-style variants (0x0F2510 / 0x0F2850 — engage with the
engine OFF, read the mode back on the DSC) and the F30 N57's **F-series DSC_25 (0x0F1860)** —
engage with the engine RUNNING, read the mode back on the LMV transfer case (0x19). The DSC_25
path is factory-confirmed from ISTA dsc_25.prg (DBE1 = LMV_FUNKTION, SERVICE 22;2E) + lmv_01.prg.
Still your car, your risk, and one owner (upstream #68) reports 0x0F1860 doesn't engage — so
verify the read-back on the car before trusting it.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

from . import transport

DSC = 0x29          # stability/DSC module — commands the transfer case
EGS = 0x18          # transmission (gear / Park)
DME = 0x12          # engine (speed)

DID_VARIANT = 0xF150     # 22 F1 50 -> DSC variant index
DID_CAPABILITY = 0xDBE1  # 22/2E DB E1 = LMV_FUNKTION (1=AWD active, 0=RWD) — the control on both DSC gens
DID_GEAR = 0xDA2E        # EGS: 00 = Park
LMV = 0x19               # F-series transfer case ECU (separate from DSC); DSC_25 reads its mode here
DID_LMV_MODE = 0x4002    # LMV STATUS_SOLLMOMENT; data byte 2 = qualifier (0x20 AWD, 0x28/0x70 RWD, 0x60 off)

# 340i-style DSC: engage with the engine OFF, read the mode back on the DSC (DBE1).
SUPPORTED_340I = (0x0F2510, 0x0F2850)
# F30 N57's F-series DSC_25 (0x0F1860): engage with the engine RUNNING, read the mode back on the
# LMV. Factory-confirmed from ISTA dsc_25.prg (DBE1 = LMV_FUNKTION, SERVICE 22;2E) + lmv_01.prg.
SUPPORTED_DSC25 = (0x0F1860,)
SUPPORTED_VARIANTS = SUPPORTED_340I + SUPPORTED_DSC25

RWD, AWD = 0, 1


@dataclass
class Interlock:
    ok: bool
    reasons: List[str]


def check_interlock(speed_kmh: Optional[int], in_park: Optional[bool], engine_running: Optional[bool],
                    require_park: bool = True, require_engine_off: bool = True,
                    require_engine_running: bool = False) -> Interlock:
    """The software fuse. Pure — unit-testable without a car.

    Standstill is mandatory for engaging RWD (never bypassable). Park is required by default.
    Engine state depends on the DSC generation: 340i-style wants engine-OFF (a code-enforced
    default, waivable); F-series DSC_25 wants engine-RUNNING (not waivable — the write proves
    nothing with the engine off). An unreadable condition refuses (fail-safe).
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
    if require_engine_running:
        if engine_running is None:
            reasons.append("could not read engine state — refusing")
        elif not engine_running:
            reasons.append("engine must be RUNNING for this DSC_25 variant — start it")
    elif require_engine_off:
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


def read_mode_lmv(hsfz) -> Optional[str]:
    """DSC_25: read the transfer-case mode from the LMV (0x19), DID 4002 data-byte 2 (qualifier):
    0x20 = AWD (DSC target applied), 0x28/0x70 = RWD, 0x60 = engine-off fallback (unknown)."""
    try:
        d = transport.did(hsfz, LMV, DID_LMV_MODE)
    except transport.DiagError:
        return None
    if len(d) < 3:
        return None
    return "AWD" if d[2] == 0x20 else "RWD" if d[2] in (0x28, 0x70) else None


def status(hsfz) -> dict:
    var = _variant(hsfz)
    er = read_engine_running(hsfz)
    dsc25 = var in SUPPORTED_DSC25
    return {
        "variant": var,
        "supported": var in SUPPORTED_VARIANTS,
        "mode": read_mode_lmv(hsfz) if dsc25 else read_mode(hsfz),
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
    dsc25 = var in SUPPORTED_DSC25

    if target == RWD:
        # DSC_25 engages with the engine RUNNING (off makes the write prove nothing); the 340i
        # path wants it off. Standstill + Park hold either way.
        lock = check_interlock(read_speed(hsfz), read_park(hsfz), read_engine_running(hsfz),
                               require_park,
                               require_engine_off=False if dsc25 else require_engine_off,
                               require_engine_running=dsc25)
        if not lock.ok:
            return False, "INTERLOCK: " + "; ".join(lock.reasons)
    elif var not in (0x0F2850,) + SUPPORTED_DSC25:
        # AWD restore: a live write exists on 0x0F2850 and DSC_25 (dsc_25.prg: DBE1=1 restores,
        # owner-confirmed); other variants only restore on an ignition cycle.
        return False, "this DSC restores AWD on an ignition cycle (no live write for 0x%06X)" % var

    plan = "2E DBE1 %02X  (%s) on DSC 0x29" % (target, "RWD" if target == RWD else "AWD")
    if not commit:
        return False, "DRY RUN — would send %s. Add --commit to perform it." % plan

    # NB: caller is expected to have enforced the VIN consent gate before commit.
    try:
        transport.raw(hsfz, DSC, bytes([0x2E, (DID_CAPABILITY >> 8) & 0xFF, DID_CAPABILITY & 0xFF, target]))
    except transport.DiagError as e:
        return False, "DSC refused the write (%s)" % e
    now = read_mode_lmv(hsfz) if dsc25 else read_mode(hsfz)
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
    # F-series DSC_25 (0x0F1860): engine must be RUNNING; standstill + Park still hold
    assert check_interlock(0, True, True, require_engine_off=False, require_engine_running=True).ok
    assert not check_interlock(0, True, False, require_engine_off=False, require_engine_running=True).ok  # off -> blocked
    assert not check_interlock(5, True, True, require_engine_off=False, require_engine_running=True).ok   # moving -> blocked
    assert not check_interlock(0, True, None, require_engine_off=False, require_engine_running=True).ok   # unknown -> refuse
    return True


if __name__ == "__main__":
    selftest()
    print("xDrive interlock selftest OK")
