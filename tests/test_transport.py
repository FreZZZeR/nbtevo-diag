"""Offline transport + DTC tests against a canned gateway (FakeHsfz) — no car.

Covers the paths test_signals.py doesn't: the UDS/OBD request-reply plumbing in
transport.py and the 19 02 FF parse in dtc.py. Needs evocode importable (transport
imports it); the dev checkout at ~/Documents/evocode or an installed evocode both work.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nbtevo_diag import dtc, transport, signals  # noqa: E402
from fake_gateway import FakeHsfz  # noqa: E402

# One canned 19 02 FF response, shared in SHAPE with the id6 Codes worker test
# (native/tests/test_codes.c): DTC 0x101A05 status 0x08 (confirmed), 0x480011 status 0x2E.
SHARED_DTCS = [(0x101A05, 0x08), (0x480011, 0x2E)]


def test_read_dtcs_parses():
    gw = FakeHsfz().on_dtcs(signals.DME, SHARED_DTCS)
    got = dtc.read_dtcs(gw, signals.DME)
    assert [(d.code, d.status) for d in got] == SHARED_DTCS
    assert got[0].hex == "101A05"
    assert got[0].active and "confirmed" in got[0].flags()


def test_read_dtcs_empty():
    gw = FakeHsfz().on_dtcs(signals.EGS, [])
    assert dtc.read_dtcs(gw, signals.EGS) == []


def test_read_dtcs_rejects_wrong_service():
    # a non-negative reply that isn't `59 02 ..` must be rejected by dtc.py's guard
    gw = FakeHsfz().on(signals.DME, b"\x19\x02\xff", b"\x62\x00\x00")
    try:
        dtc.read_dtcs(gw, signals.DME)
    except transport.DiagError:
        pass
    else:
        raise AssertionError("malformed DTC reply should raise")


def test_read_signal_obd():
    # coolant: OBD PID 0x05, decode raw-40; raw 0x50 = 80 -> 40.0 degC (in bounds)
    cool = signals.BY_KEY["coolant"]
    gw = FakeHsfz().on_obd(cool.ecu, cool.addr, bytes([0x50]))
    r = transport.read_signal(gw, cool)
    assert r.ok and r.value == 40.0


def test_read_signal_did():
    # trans_temp: UDS DID 0xDA12 on EGS, decode raw-48; raw 0x0050 = 80 -> 32.0 degC
    tt = signals.BY_KEY["trans_temp"]
    gw = FakeHsfz().on_did(tt.ecu, tt.addr, bytes([0x00, 0x50]))
    r = transport.read_signal(gw, tt)
    assert r.ok and r.value == 32.0


def test_nrc_unknown_request_raises():
    gw = FakeHsfz()  # nothing registered -> 7F serviceNotSupported
    try:
        dtc.read_dtcs(gw, signals.DSC if hasattr(signals, "DSC") else 0x29)
    except transport.DiagError:
        pass
    else:
        raise AssertionError("unregistered request should raise")


def test_clear_dtcs_gated():
    gw = FakeHsfz().on(signals.DME, b"\x14\xff\xff\xff", b"\x54")
    assert dtc.clear_dtcs(gw, signals.DME, commit=False) is False   # dry-run: no write
    assert dtc.clear_dtcs(gw, signals.DME, commit=True) is True     # 14 FF FF FF -> 54


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok", name)
    print("all tests passed")
