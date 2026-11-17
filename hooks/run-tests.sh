#!/bin/sh
# Local verification stage for nbtevo-diag (run by pre-push).
set -e
python3 tests/test_signals.py
python3 tests/test_transport.py
