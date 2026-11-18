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
from .signals import ALL, UDS, Signal, by_confidence


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


# What to try first on the car comes from the catalog (signals.py) — the single source of
# truth — not a second hand-kept list here. Each `bmw-candidate` is a B58-gas DID with a known
# address and its own ECU (e.g. transmission temp is on EGS 0x18, not the DME): the prover reads
# each to see which answer on the N57, then you validate the scaling and promote it to a map.
# AFR is deliberately absent from the catalog; ignition timing is present but flagged N/A-diesel.
def candidates() -> List[Signal]:
    """Catalog signals worth probing first: BMW UDS DIDs with a known address."""
    return [s for s in by_confidence("bmw-candidate") if s.service == UDS and s.addr]


def hunt(hsfz) -> dict:
    """Probe the catalog's candidate DIDs, each on its OWN ECU; {group: [DidHit...]}.
    Addresses and ECUs are read from signals.py, so this can't drift from the catalog."""
    out: dict = {}
    for s in candidates():
        out.setdefault(s.group, []).extend(scan_dids(hsfz, s.ecu, [s.addr]))
    return out


def selftest():
    """Pure checks that don't touch a car."""
    assert any(s.confidence == "probe" for s in ALL)
    cand = {(s.ecu, s.addr) for s in candidates()}
    # the two the old hand-list silently dropped must now be covered — trans temp on the EGS:
    assert (0x12, 0x580F) in cand, "intake-air candidate 0x580F missing from the hunt"
    assert (0x18, 0xDA12) in cand, "trans temp must hunt on EGS 0x18, not the DME"
    assert all(addr for _ecu, addr in cand), "a candidate leaked a 0x0000 placeholder address"
    return True


if __name__ == "__main__":
    selftest()
    cand = candidates()
    print("probe selftest OK; %d catalog candidates across groups: %s"
          % (len(cand), sorted({s.group for s in cand})))
