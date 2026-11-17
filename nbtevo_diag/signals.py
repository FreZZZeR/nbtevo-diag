"""The signal catalog — the single source of truth for what the app reads.

Each Signal says: which ECU, how to ask (standard OBD mode-01 PID, or a BMW UDS DID),
how to decode the raw bytes to a number, its unit, plausibility bounds, and — crucially —
a *confidence*:

  standard       SAE J1979 OBD-II mode 01. Works on the N57 diesel too; formulas are
                 standardised, so these are high-confidence and verifiable immediately.
  computed       derived from other signals (boost = MAP - baro; power from torque x rpm).
  bmw-candidate  a BMW-specific UDS DID taken from the B58/340i GASOLINE reference
                 (ID6_apps). A *template*, NOT valid for the N57 until validated on the car.
  probe          a signal we want but don't have an address/decode for yet (oil pressure,
                 rail pressure, turbo speed/VGT, DPF soot/regen, AdBlue, pad wear, ...).
                 The prober finds and validates these; until then the app shows "n/a".

Nothing here is trusted on the car until `verified` is set by validating the reading
against an independent source (ISTA / cluster / a known logger). A wrong-but-plausible
number is worse than "n/a".

Decode functions take the data bytes AFTER the service/PID or DID echo (e.g. for a mode-01
reply `41 05 7B`, decode receives b"\\x7b"; for `62 43 00 ..`, the bytes after 62 43 00).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, Optional

# --- ECU diagnostic addresses (target byte; the FEM gateway 0x10 routes by this) ---
DME = 0x12   # engine (N57 diesel = DDE)
EGS = 0x18   # automatic transmission (ZF 8HP)
HEAD = 0x63  # NBT EVO head unit
FEM = 0x10   # body gateway
# pad-wear / TPMS / CBS live in chassis/body modules whose address we confirm by probing.

OBD = "obd"  # service 01 (mode-01 PID)
UDS = "uds"  # service 22 (ReadDataByIdentifier)


def _u8(d, i=0):  return d[i]
def _u16(d, i=0): return (d[i] << 8) | d[i + 1]
def _s16(d, i=0):
    v = _u16(d, i)
    return v - 0x10000 if v & 0x8000 else v


@dataclass
class Dec:
    """Declarative affine decode: value = raw * num / den + offset, where raw is the
    big-endian integer in the first ``width`` bytes (two's-complement if ``signed``).

    This is the single source of truth for decoding a single DID/PID: the Python reader
    calls it, and ``codegen/gen_dids.py`` (in nbtevo-id6-apps) emits these same fields into
    the C table ``dids.h``. Change a scale here and the native collector follows — no drift.
    """
    width: int
    num: int = 1
    den: int = 1
    offset: float = 0.0
    signed: bool = False

    def __call__(self, d) -> float:
        if len(d) < self.width:
            raise ValueError("short data: need %d bytes" % self.width)
        raw = int.from_bytes(bytes(d[: self.width]), "big", signed=self.signed)
        return raw * self.num / self.den + self.offset


@dataclass
class Signal:
    key: str
    label: str
    ecu: int
    service: str                 # OBD or UDS
    addr: int                    # PID (OBD) or DID (UDS)
    decode: Optional[Callable[[bytes], Optional[float]]]
    unit: str = ""
    group: str = "misc"
    confidence: str = "probe"    # standard | computed | bmw-candidate | probe
    lo: Optional[float] = None   # plausibility bounds; outside => treat as invalid/"n/a"
    hi: Optional[float] = None
    note: str = ""
    verified: bool = False       # set true only after validating on THIS car

    def req(self) -> bytes:
        """The UDS/OBD request bytes for this signal."""
        if self.service == OBD:
            return bytes([0x01, self.addr])
        return bytes([0x22, (self.addr >> 8) & 0xFF, self.addr & 0xFF])

    def plausible(self, v) -> bool:
        if v is None or not isinstance(v, (int, float)):
            return v is not None
        if self.lo is not None and v < self.lo:
            return False
        if self.hi is not None and v > self.hi:
            return False
        return True


def _sig(*a, **k) -> Signal:
    return Signal(*a, **k)


# ---------------------------------------------------------------------------
# Standard OBD-II mode 01 (SAE J1979) — works on the diesel; verify-on-car but high confidence
# ---------------------------------------------------------------------------
STANDARD = [
    _sig("coolant", "Coolant temp", DME, OBD, 0x05, Dec(1, offset=-40), "°C", "engine_temp", "standard", -40, 215),
    _sig("oil_temp", "Oil temp", DME, OBD, 0x5C, Dec(1, offset=-40), "°C", "engine_temp", "standard", -40, 210),
    _sig("iat", "Intake air temp", DME, OBD, 0x0F, Dec(1, offset=-40), "°C", "engine_temp", "standard", -40, 215),
    _sig("ambient", "Ambient temp", DME, OBD, 0x46, Dec(1, offset=-40), "°C", "ambient", "standard", -40, 215),
    _sig("rpm", "Engine RPM", DME, OBD, 0x0C, Dec(2, den=4), "rpm", "driveline", "standard", 0, 6000),
    _sig("speed", "Vehicle speed", DME, OBD, 0x0D, Dec(1), "km/h", "driveline", "standard", 0, 320),
    _sig("map", "Manifold abs press", DME, OBD, 0x0B, Dec(1), "kPa", "forced_induction", "standard", 0, 400),
    _sig("baro", "Barometric press", DME, OBD, 0x33, Dec(1), "kPa", "ambient", "standard", 50, 110),
    _sig("maf", "Mass air flow", DME, OBD, 0x10, Dec(2, den=100), "g/s", "forced_induction", "standard", 0, 655),
    _sig("load", "Engine load", DME, OBD, 0x04, Dec(1, num=100, den=255), "%", "performance", "standard", 0, 100),
    _sig("throttle", "Throttle pos", DME, OBD, 0x11, Dec(1, num=100, den=255), "%", "performance", "standard", 0, 100),
    _sig("pedal", "Accelerator pedal", DME, OBD, 0x5A, Dec(1, num=100, den=255), "%", "performance", "standard", 0, 100),
    _sig("timing", "Timing advance", DME, OBD, 0x0E, Dec(1, den=2, offset=-64), "°", "performance", "standard", -64, 64),
    _sig("torque_pct", "Actual torque", DME, OBD, 0x62, Dec(1, offset=-125), "%", "performance", "standard", -125, 130),
    _sig("ref_torque", "Reference torque", DME, OBD, 0x63, Dec(2), "Nm", "performance", "standard", 0, 2000),
    _sig("fuel_rate", "Fuel rate", DME, OBD, 0x5E, Dec(2, den=20), "L/h", "fuel", "standard", 0, 400),
    _sig("fuel_level", "Fuel level", DME, OBD, 0x2F, Dec(1, num=100, den=255), "%", "fuel", "standard", 0, 100),
    _sig("ctrl_voltage", "Module voltage", DME, OBD, 0x42, Dec(2, den=1000), "V", "electrical", "standard", 0, 30),
    _sig("egr_cmd", "Commanded EGR", DME, OBD, 0x2C, Dec(1, num=100, den=255), "%", "aftertreatment", "standard", 0, 100),
    _sig("run_time", "Engine run time", DME, OBD, 0x1F, Dec(2), "s", "misc", "standard", 0, 65535),
]

# ---------------------------------------------------------------------------
# Computed — the "sport displays" numbers and gauge conveniences
# ---------------------------------------------------------------------------
# These are resolved by the live engine from other signals (see cli/live). Kept here as
# definitions + the pure math so they're documented and testable.
def boost_bar(map_kpa: float, baro_kpa: float) -> float:
    """Gauge boost (above atmosphere), bar."""
    return max(0.0, (map_kpa - baro_kpa)) / 100.0


def torque_nm(ref_torque_nm: float, actual_pct: float) -> float:
    return ref_torque_nm * actual_pct / 100.0


def power_kw(torque_nm_: float, rpm: float) -> float:
    return torque_nm_ * rpm * 2 * math.pi / 60.0 / 1000.0


COMPUTED = [
    _sig("boost", "Boost (gauge)", DME, OBD, 0x00, None, "bar", "forced_induction", "computed", 0, 3,
         note="= (MAP - baro)/100; also show psi = bar*14.5"),
    _sig("torque", "Torque", DME, OBD, 0x00, None, "Nm", "performance", "computed", 0, 1200,
         note="= reference_torque * actual_torque%/100"),
    _sig("power", "Power", DME, OBD, 0x00, None, "kW", "performance", "computed", 0, 500,
         note="= torque * rpm * 2pi/60/1000; show PS = kW*1.36"),
]

# ---------------------------------------------------------------------------
# BMW-specific UDS DIDs — B58/340i GASOLINE reference (ID6_apps). TEMPLATE for the N57,
# NOT valid until validated. Decodes are the gas scalings; keep for the prober to try.
# ---------------------------------------------------------------------------
BMW_CANDIDATE = [
    _sig("boost_abs_bmw", "Boost abs (BMW)", DME, UDS, 0x4205, Dec(2, den=128), "kPa", "forced_induction", "bmw-candidate", note="B58 gas; N57 unverified"),
    _sig("ambient_bmw", "Ambient abs (BMW)", DME, UDS, 0x4201, Dec(2, den=256), "kPa", "ambient", "bmw-candidate", note="B58 gas; N57 unverified"),
    _sig("rail_bmw", "Rail pressure (BMW)", DME, UDS, 0x56D7, Dec(2, den=50), "bar", "pressure", "bmw-candidate", note="B58 ~250bar GDI; N57 common-rail ~2000bar — SCALE WRONG, re-derive"),
    _sig("oil_press_bmw", "Oil pressure (BMW)", DME, UDS, 0x586F, Dec(2, den=1000), "bar", "pressure", "bmw-candidate", note="B58 gas; N57 may lack a continuous sensor"),
    _sig("coolant_bmw", "Coolant (BMW)", DME, UDS, 0x4300, Dec(2, den=10, offset=-273.15), "°C", "engine_temp", "bmw-candidate", note="B58 gas; N57 unverified"),
    _sig("iat_bmw", "Intake air (BMW)", DME, UDS, 0x580F, Dec(1, num=3, den=4, offset=-48), "°C", "engine_temp", "bmw-candidate", note="B58 gas; N57 unverified (0.75 = 3/4)"),
    _sig("timing_bmw", "Ignition timing (BMW)", DME, UDS, 0x4A49, Dec(2, den=10, signed=True), "°", "performance", "bmw-candidate", note="spark concept; N/A on diesel"),
    _sig("trans_temp", "Transmission temp", EGS, UDS, 0xDA12, Dec(2, offset=-48), "°C", "driveline", "bmw-candidate", note="EGS/ZF 8HP; most likely to carry to N57 — still validate"),
]

# ---------------------------------------------------------------------------
# Probe targets — wanted, but address/decode unknown. The prober discovers these on the car.
# decode=None => shown as "n/a" until found. ecu/addr are best guesses to seed the scan.
# ---------------------------------------------------------------------------
PROBE = [
    _sig("rail_diesel", "Rail pressure (diesel)", DME, UDS, 0x0000, None, "bar", "pressure", "probe", note="common-rail 0..~2000 bar; probe DDE"),
    _sig("oil_press", "Oil pressure", DME, UDS, 0x0000, None, "bar", "pressure", "probe", note="may be switch-only on N57; confirm a continuous sensor exists"),
    _sig("turbo_speed", "Turbo speed", DME, UDS, 0x0000, None, "rpm", "forced_induction", "probe", note="twin-turbo N57; may not be exposed"),
    _sig("vgt_pos", "VGT / wastegate pos", DME, UDS, 0x0000, None, "%", "forced_induction", "probe"),
    _sig("injection_qty", "Injection quantity", DME, UDS, 0x0000, None, "mg/stk", "fuel", "probe"),
    _sig("egt", "Exhaust gas temp", DME, UDS, 0x0000, None, "°C", "aftertreatment", "probe", note="diesel; per-sensor; OBD PID 0x78/0x98 may also work"),
    _sig("dpf_soot", "DPF soot load", DME, UDS, 0x0000, None, "g", "aftertreatment", "probe", note="diesel-critical"),
    _sig("dpf_regen", "DPF regen active", DME, UDS, 0x0000, None, "bool", "aftertreatment", "probe"),
    _sig("dpf_dp", "DPF diff pressure", DME, UDS, 0x0000, None, "mbar", "aftertreatment", "probe"),
    _sig("scr_level", "AdBlue/SCR level", DME, UDS, 0x0000, None, "%", "aftertreatment", "probe", note="335d has SCR"),
    _sig("pad_front", "Brake pad wear front", FEM, UDS, 0x0000, None, "state", "chassis", "probe", note="NOT DME — chassis/CBS; confirm module+addr"),
    _sig("pad_rear", "Brake pad wear rear", FEM, UDS, 0x0000, None, "state", "chassis", "probe", note="NOT DME — chassis/CBS"),
    _sig("tpms", "Tyre pressures (RDC)", FEM, UDS, 0x0000, None, "kPa", "chassis", "probe", note="RDC/TPMS module"),
    _sig("cbs", "CBS service items", FEM, UDS, 0x0000, None, "various", "chassis", "probe", note="oil/brake-fluid/inspection remaining"),
]

ALL = STANDARD + COMPUTED + BMW_CANDIDATE + PROBE
BY_KEY = {s.key: s for s in ALL}


def by_group(group: str):
    return [s for s in ALL if s.group == group]


def by_confidence(conf: str):
    return [s for s in ALL if s.confidence == conf]


def groups():
    seen = []
    for s in ALL:
        if s.group not in seen:
            seen.append(s.group)
    return seen


def selftest():
    """Pure, no-car self-check of the catalog and decode math."""
    # keys unique
    assert len(BY_KEY) == len(ALL), "duplicate signal keys"
    # every non-probe/computed signal has a decode; requests are well-formed
    for s in ALL:
        r = s.req()
        assert r[0] in (0x01, 0x22), s.key
        if s.confidence in ("standard", "bmw-candidate"):
            assert callable(s.decode), f"{s.key} needs a decode"
    # decode spot-checks (standard formulas)
    assert BY_KEY["coolant"].decode(b"\x7b") == 83            # 0x7B-40
    assert BY_KEY["rpm"].decode(b"\x1a\xf8") == 1726.0        # 0x1AF8/4
    assert BY_KEY["torque_pct"].decode(b"\xa0") == 35         # 0xA0-125
    assert BY_KEY["ctrl_voltage"].decode(b"\x37\x74") == 14.196
    # computed math
    assert abs(boost_bar(230, 100) - 1.3) < 1e-9
    tq = torque_nm(600, 50)
    assert tq == 300.0
    assert abs(power_kw(300, 4000) - 125.664) < 1e-2
    # plausibility gate
    assert BY_KEY["coolant"].plausible(90) and not BY_KEY["coolant"].plausible(900)
    return True


if __name__ == "__main__":
    selftest()
    print(f"catalog OK: {len(ALL)} signals, {len(groups())} groups")
    for g in groups():
        print(f"  {g:16} " + ", ".join(s.key for s in by_group(g)))
