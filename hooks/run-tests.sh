#!/bin/sh
# Local verification stage for nbtevo-diag (run by pre-push).
exec python3 tests/test_signals.py
