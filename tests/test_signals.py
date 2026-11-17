"""Offline checks — no car, no evocode needed (signals/probe are pure)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nbtevo_diag import signals, probe, control  # noqa: E402


def test_catalog_selftest():
    assert signals.selftest() is True


def test_probe_selftest():
    assert probe.selftest() is True


def test_standard_signals_have_bounds_and_decode():
    for s in signals.by_confidence("standard"):
        assert s.decode is not None, s.key
        assert s.service in (signals.OBD, signals.UDS)


def test_computed_math():
    assert signals.boost_bar(250, 100) == 1.5
    assert signals.torque_nm(500, 80) == 400.0
    assert round(signals.power_kw(400, 3000), 1) == 125.7


def test_xdrive_fuse():
    assert control.selftest() is True
    # check_interlock(speed, in_park, engine_running, ...)
    assert control.check_interlock(0, True, False).ok            # stopped + Park + engine off -> allowed
    assert not control.check_interlock(30, True, False).ok       # moving -> blocked
    assert not control.check_interlock(0, False, False).ok       # not in Park -> blocked
    assert not control.check_interlock(0, True, True).ok         # engine running -> blocked by default
    assert control.check_interlock(0, True, True, require_engine_off=False).ok  # waived consciously


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok", name)
    print("all tests passed")
