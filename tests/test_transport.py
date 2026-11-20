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


class AppendingDde:
    """A DDE where 2C 01 APPENDS to an already-defined dynamic DID (ISO 14229 allows it) and
    only 2C 03 empties it. Reading two signals in a row is right only if we clear first."""

    def __init__(self, values):                 # {source DID: its data bytes}
        self.values, self.defined = values, []

    def request(self, ecu, uds, p2_star=6.0):
        uds = bytes(uds)
        if uds[:2] == b"\x2c\x03":                                   # clear
            self.defined = []
            return b"\x6c\x03" + uds[2:4]
        if uds[:2] == b"\x2c\x01" and len(uds) == 8:                 # defineByIdentifier
            src = (uds[4] << 8) | uds[5]
            if src not in self.values:
                return bytes([0x7F, 0x2C, 0x31])
            self.defined.append((src, uds[6], uds[7]))
            return b"\x6c\x01" + uds[2:4]
        if uds == b"\x22\xf3\x03" and self.defined:
            return b"\x62\xf3\x03" + b"".join(self.values[s][p - 1:p - 1 + n] for s, p, n in self.defined)
        return bytes([0x7F, uds[0], 0x31])      # incl. a direct 22 <did> and the 2C 02 memory form


def test_read_signal_dyn():
    # N57 DDE DIDs answer only through a dynamic DID: clear (2C 03), define by identifier
    # (2C 01), read 22 F303 — never a direct 22, never the memory-address form (2C 02).
    dde = AppendingDde({0x4517: b"\x37\x8c", 0x461B: b"\x2e\xe0", 0x4746: b"\x27\x10"})
    oil, cool, rail = (signals.BY_KEY[k] for k in ("oil_temp_dde", "coolant_dde", "rail_dde"))
    r = transport.read_signal(dde, oil)
    assert r.ok and abs(r.value - 42.2) < 1e-9, r                      # 0x378C/100-100
    r = transport.read_signal(dde, cool)                               # must NOT be the oil bytes again
    assert r.ok and abs(r.value - 20.0) < 1e-9, r                      # 0x2EE0/100-100
    r = transport.read_signal(dde, rail)
    assert r.ok and abs(r.value - 457.77) < 1e-6, r                    # 0x2710*0.045777 (factory)
    assert abs(transport.read_signal(dde, oil).value - 42.2) < 1e-9    # and back again
    # an ECU that refuses the clear ("nothing defined yet") still reads: only the define must succeed
    gw = (FakeHsfz()
          .on(oil.ecu, b"\x2c\x01\xf3\x03\x45\x17\x01\x02", b"\x6c\x01\xf3\x03")
          .on(oil.ecu, b"\x22\xf3\x03", b"\x62\xf3\x03\x37\x8c"))
    assert transport.read_signal(gw, oil).ok
    # a refused define surfaces as not-ok (no crash), and so does a source DID the ECU lacks
    assert not transport.read_signal(FakeHsfz(), oil).ok
    assert not transport.read_signal(AppendingDde({}), oil).ok


def test_n57_computed_gauges():
    # no mode-01 on the N57: boost/power come from the DDE's own DIDs
    from nbtevo_diag import cli
    c = cli._resolve_computed({"boost_abs_dde": 2500.0, "baro_dde": 1000.0,
                               "torque_dde": 600.0, "rpm_dde": 3000.0})
    assert c["boost"] == 1.5, c                                   # (2500-1000) hPa = 1.5 bar
    assert c["torque"] == 600.0
    assert c["power"] == round(signals.power_kw(600.0, 3000.0), 1)  # ~188.5 kW


def test_xdrive_dsc25_engage():
    # F30 N57 DSC_25 (0x0F1860): engine RUNNING, stopped, Park -> engage RWD, read back on the LMV.
    from nbtevo_diag import control as c
    gw = (FakeHsfz()
          .on_did(c.DSC, c.DID_VARIANT, bytes([0x0F, 0x18, 0x60]))   # variant 0x0F1860
          .on_obd(c.DME, 0x0D, bytes([0]))                            # speed 0
          .on_did(c.EGS, c.DID_GEAR, bytes([0]))                      # Park
          .on_obd(c.DME, 0x0C, bytes([0x0A, 0x00]))                   # rpm != 0 -> engine running
          .on(c.DSC, b"\x2e\xdb\xe1\x00", b"\x6e\xdb\xe1")            # 2E DBE1 00 -> positive
          .on_did(c.LMV, c.DID_LMV_MODE, bytes([0x09, 0xc4, 0x70])))  # LMV byte-2 0x70 -> RWD
    st = c.status(gw)
    assert st["variant"] == 0x0F1860 and st["supported"] and st["mode"] == "RWD", st
    done, msg = c.set_mode(gw, c.RWD, commit=True)
    assert done, msg
    # engine OFF must block DSC_25 engage (engine-running is required, not waivable)
    off = (FakeHsfz()
           .on_did(c.DSC, c.DID_VARIANT, bytes([0x0F, 0x18, 0x60]))
           .on_obd(c.DME, 0x0D, bytes([0]))
           .on_did(c.EGS, c.DID_GEAR, bytes([0]))
           .on_obd(c.DME, 0x0C, bytes([0, 0])))                       # rpm 0 -> engine off
    done2, msg2 = c.set_mode(off, c.RWD, commit=True)
    assert not done2 and "RUNNING" in msg2, msg2


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
