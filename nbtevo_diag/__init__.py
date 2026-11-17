"""nbtevo-diag — on-head-unit diagnostics for the BMW NBT EVO (F30).

Phase 0 (this code): a read-only data engine on evocode's HSFZ/UDS transport —
module DTC read/clear and a live-signal catalog + prober, run from a dev machine
over ENET to discover and validate the car's real DIDs (the N57 map).

Phase 1 (later, needs the QNX toolchain): port this engine into a native on-head-unit
collector + HMI plugin so it runs from iDrive with no laptop and no OBD adapter.
"""
__version__ = "0.0.1"
