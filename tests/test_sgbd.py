"""Offline check — no car, no .prg file needed (the SGBD walk is pure on a synthetic table)."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nbtevo_diag import sgbd  # noqa: E402


def test_sgbd_demo():
    assert sgbd._demo() is True


if __name__ == "__main__":
    test_sgbd_demo()
    print("ok test_sgbd_demo")
