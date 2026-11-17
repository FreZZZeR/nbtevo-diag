"""Signal discovery — turn `probe`/`bmw-candidate` guesses into a verified N57 map.

Two tools:
  probe_catalog  read every signal that already has a decoder, show raw + decoded +
                 whether it's plausible. Lets you eyeball which candidates actually answer.
  scan_dids      sweep a range of UDS DIDs on one ECU and report which ones answer (0x62)
                 vs. reject (NRC). This is how we FIND the diesel's real DIDs — then you
                 map a DID to a signal, work out the scaling, and record it as verified
                 against an independent reference (ISTA / cluster / a logger).

Discovered facts belong in maps/<vin-or-car>.json (gitignored) — never hardcode a car's map
into the catalog; the catalog holds method + candidates, the map holds one car's verified truth.
"""
from __future__ import annotations

from dataclasses import dataclass, asdict
from typing import Iterable, List, Optional

try:
    from . import transport
except ImportError:  # allow running this file directly for the pure selftest
    transport = None
from .signals import ALL, Signal, by_confidence


@dataclass
class DidHit:
    did: int
    ok: bool
    raw: str = ""     # hex of the data bytes after 62 hi lo
    note: str = ""


def probe_catalog(hsfz, sigs: Optional[Iterable[Signal]] = None):
    """Read all readable signals (those with a decoder). Returns [Reading]."""
    sigs = list(sigs) if sigs is not None else [s for s in ALL if s.decode is not None]
    return [transport.read_signal(hsfz, s) for s in sigs]


def scan_dids(hsfz, ecu: int, dids: Iterable[int]) -> List[DidHit]:
    """Try reading each DID on one ECU; keep the ones that answer positively."""
    hits: List[DidHit] = []
    for d in dids:
        try:
            data = transport.did(hsfz, ecu, d)
            hits.append(DidHit(d, True, data.hex()))
        except transport.DiagError as e:
            # NRC 0x31 (requestOutOfRange) = DID not supported — expected noise, skip quietly
            msg = str(e)
            if "0x31" not in msg:
                hits.append(DidHit(d, False, "", msg))
        except Exception as e:  # noqa: BLE001
            hits.append(DidHit(d, False, "", str(e)))
    return [h for h in hits if h.ok or h.note]


def answering_dids(hsfz, ecu: int, start: int = 0x4000, end: int = 0x5FFF) -> List[DidHit]:
    """Sweep a BMW DID band (DME live values typically cluster in 0x40xx..0x5Fxx)."""
    return scan_dids(hsfz, ecu, range(start, end + 1))


# candidate DIDs worth trying first for the N57, grouped by what we're hunting.
# (These are hypotheses — many come from the B58 gas reference; prove each on the car.)
N57_HUNT = {
    "boost/ambient": [0x4205, 0x4201, 0x4200, 0x4206],
    "rail_pressure": [0x56D7, 0x4432, 0x4433],
    "oil_pressure": [0x586F, 0x4A50],
    "coolant/oil_temp": [0x4300, 0x4301, 0x4A40],
    "turbo/vgt": [0x4A30, 0x4A31, 0x4A32],
    "dpf/scr": [0x4B00, 0x4B01, 0x4C00],
}


def hunt(hsfz, ecu: int = 0x12) -> dict:
    """Try the curated N57 hunt list; return {group: [DidHit...]}."""
    return {g: scan_dids(hsfz, ecu, dids) for g, dids in N57_HUNT.items()}


def selftest():
    """Pure checks that don't touch a car."""
    assert any(s.confidence == "probe" for s in ALL)
    assert all(isinstance(d, int) for ds in N57_HUNT.values() for d in ds)
    return True


if __name__ == "__main__":
    selftest()
    print("probe selftest OK; hunt groups:", list(N57_HUNT))
