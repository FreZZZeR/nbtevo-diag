"""nbtevo-diag CLI (Phase 0, dev machine over ENET).

  nbtevo-diag catalog                      # list the signal catalog (offline, no car)
  nbtevo-diag modules --host IP            # enumerate awake ECUs
  nbtevo-diag dtc     --host IP [--module 0x12]            # read fault memory
  nbtevo-diag dtc     --host IP --module 0x12 --clear --commit   # clear (gated)
  nbtevo-diag live    --host IP [keys...]  # poll + display live signals
  nbtevo-diag probe   --host IP [--group G]# read readable signals, show raw+decoded
  nbtevo-diag scan    --host IP --ecu 0x12 [--hunt | --band 4000-5FFF]  # discover DIDs

--host is the FEM gateway IP from discovery (evocode: `python -m evocode ...` or the ENET
ident broadcast). Everything is read-only except `dtc --clear --commit`.
"""
from __future__ import annotations

import argparse
from typing import Optional

from . import control
from . import dtc as dtcmod
from . import probe as probemod
from . import signals as sig
from . import transport


def _host(ap):
    ap.add_argument("--host", required=True, help="FEM gateway IP (ENET)")
    ap.add_argument("--timeout", type=float, default=15.0)


def cmd_catalog(a) -> int:
    for g in sig.groups():
        print("\n== %s ==" % g)
        for s in sig.by_group(g):
            star = {"standard": "✓", "computed": "ƒ", "bmw-candidate": "?", "probe": "·"}[s.confidence]
            addr = "OBD %02X" % s.addr if s.service == sig.OBD else ("DID %04X" % s.addr if s.addr else "—")
            print("  %s %-14s %-22s %-9s %-5s %s" % (star, s.key, s.label, addr, s.unit, s.note))
    print("\nlegend: ✓ standard OBD · ƒ computed · ? BMW-candidate (validate on N57) · · probe (discover)")
    return 0


def cmd_modules(a) -> int:
    h = transport.connect(a.host, a.timeout)
    try:
        for addr, name in transport.discover_modules(h):
            print("0x%02X  %s" % (addr, name))
    finally:
        h.close()
    return 0


def cmd_dtc(a) -> int:
    h = transport.connect(a.host, a.timeout)
    try:
        if a.clear:
            ecu = int(a.module, 0)
            done = dtcmod.clear_dtcs(h, ecu, commit=a.commit)
            print("cleared 0x%02X" % ecu if done else "dry-run (add --commit to clear 0x%02X)" % ecu)
            return 0
        modules = [(int(a.module, 0), sig.BY_KEY.get("", None) and "" or transport.KNOWN_MODULES.get(int(a.module, 0), "?"))] \
            if a.module else transport.discover_modules(h)
        results = dtcmod.scan_all(h, modules)
        total = 0
        for addr, res in results.items():
            name = transport.KNOWN_MODULES.get(addr, "?")
            if isinstance(res, str):
                print("0x%02X %-22s  (%s)" % (addr, name, res))
                continue
            active = [d for d in res if d.active]
            total += len(active)
            print("0x%02X %-22s  %d DTC(s), %d active" % (addr, name, len(res), len(active)))
            for d in res:
                print("      %s" % d)
        print("\ntotal active DTCs: %d" % total)
    finally:
        h.close()
    return 0


def _resolve_computed(vals: dict) -> dict:
    out = {}
    if "map" in vals and "baro" in vals:
        out["boost"] = round(sig.boost_bar(vals["map"], vals["baro"]), 2)
    if "ref_torque" in vals and "torque_pct" in vals:
        tq = sig.torque_nm(vals["ref_torque"], vals["torque_pct"])
        out["torque"] = round(tq, 0)
        if "rpm" in vals:
            out["power"] = round(sig.power_kw(tq, vals["rpm"]), 1)
    return out


def cmd_live(a) -> int:
    keys = a.keys or [s.key for s in sig.STANDARD]
    want = [sig.BY_KEY[k] for k in keys if k in sig.BY_KEY and sig.BY_KEY[k].decode]
    h = transport.connect(a.host, a.timeout)
    try:
        readings = probemod.probe_catalog(h, want)
        vals = {r.sig.key: r.value for r in readings if r.ok}
        for r in readings:
            v = ("%.2f %s" % (r.value, r.sig.unit)) if r.ok else ("n/a (%s)" % r.err)
            print("  %-14s %s" % (r.sig.label, v))
        comp = _resolve_computed(vals)
        if comp:
            print("  -- computed --")
            for k, v in comp.items():
                s = sig.BY_KEY[k]
                extra = " (%.1f psi)" % (v * 14.5) if k == "boost" else (" (%.0f PS)" % (v * 1.36) if k == "power" else "")
                print("  %-14s %s %s%s" % (s.label, v, s.unit, extra))
    finally:
        h.close()
    return 0


def cmd_probe(a) -> int:
    sigs = sig.by_group(a.group) if a.group else None
    h = transport.connect(a.host, a.timeout)
    try:
        for r in probemod.probe_catalog(h, sigs):
            raw = r.raw.hex() if r.raw else "-"
            v = ("%.3f %s" % (r.value, r.sig.unit)) if r.value is not None else "-"
            ok = "OK " if r.ok else "   "
            print("  %s%-14s %-8s raw=%-10s %s" % (ok, r.sig.key, v, raw, r.err))
    finally:
        h.close()
    return 0


def cmd_scan(a) -> int:
    ecu = int(a.ecu, 0)
    h = transport.connect(a.host, a.timeout)
    try:
        if a.hunt:
            for group, hits in probemod.hunt(h, ecu).items():
                print("== %s ==" % group)
                for hh in hits:
                    print("  DID %04X  %s" % (hh.did, hh.raw if hh.ok else "(%s)" % hh.note))
        else:
            lo, hi = (int(x, 16) for x in a.band.split("-"))
            for hh in probemod.scan_dids(h, ecu, range(lo, hi + 1)):
                if hh.ok:
                    print("  DID %04X  raw=%s" % (hh.did, hh.raw))
    finally:
        h.close()
    return 0


def _consent(h) -> None:
    """VIN gate — a commit write is refused unless it's the car you declared you own."""
    import os
    want = os.environ.get("NBTEVO_OWN_VIN")
    if not want:
        raise SystemExit("refusing --commit: set NBTEVO_OWN_VIN to your own car's VIN")
    try:
        vin = transport.did(h, 0x12, 0xF190).decode("latin1").strip("\x00 ")
    except Exception as e:  # noqa: BLE001
        raise SystemExit("refusing --commit: could not read VIN (%s)" % e)
    if vin != want:
        raise SystemExit("refusing --commit: connected VIN %s != NBTEVO_OWN_VIN" % vin)


def cmd_xdrive(a) -> int:
    h = transport.connect(a.host, a.timeout)
    try:
        if a.mode == "status":
            for k, v in control.status(h).items():
                print("  %-10s %s" % (k, ("0x%06X" % v) if k == "variant" and isinstance(v, int) else v))
            return 0
        target = control.RWD if a.mode == "rwd" else control.AWD
        if a.mode == "rwd":
            print("⚠ Rear-wheel drive disables the front axle — changes traction/DSC behaviour.")
            print("  Track / private land only, never on a public road. At your own risk.")
        if a.commit:
            _consent(h)  # VIN gate before any byte goes to the car
        done, msg = control.set_mode(h, target, commit=a.commit, require_park=not a.no_park_check,
                                     require_engine_off=not a.allow_running)
        print("  " + msg)
        return 0 if (done or not a.commit) else 1
    finally:
        h.close()


def main(argv: Optional[list] = None) -> int:
    ap = argparse.ArgumentParser(prog="nbtevo-diag", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("catalog").set_defaults(fn=cmd_catalog)

    p = sub.add_parser("modules"); _host(p); p.set_defaults(fn=cmd_modules)

    p = sub.add_parser("dtc"); _host(p)
    p.add_argument("--module", help="single ECU addr, e.g. 0x12 (default: all discovered)")
    p.add_argument("--clear", action="store_true"); p.add_argument("--commit", action="store_true")
    p.set_defaults(fn=cmd_dtc)

    p = sub.add_parser("live"); _host(p)
    p.add_argument("keys", nargs="*", help="signal keys (default: all standard)")
    p.set_defaults(fn=cmd_live)

    p = sub.add_parser("probe"); _host(p)
    p.add_argument("--group", choices=sig.groups())
    p.set_defaults(fn=cmd_probe)

    p = sub.add_parser("scan"); _host(p)
    p.add_argument("--ecu", default="0x12")
    p.add_argument("--hunt", action="store_true", help="try the curated N57 candidate DIDs")
    p.add_argument("--band", default="4000-5FFF", help="DID range hex, e.g. 4000-5FFF")
    p.set_defaults(fn=cmd_scan)

    p = sub.add_parser("xdrive", help="xDelete-style temporary RWD (actuation; dry-run unless --commit)")
    _host(p)
    p.add_argument("mode", choices=["status", "rwd", "awd"])
    p.add_argument("--commit", action="store_true", help="perform the write (VIN-gated)")
    p.add_argument("--no-park-check", action="store_true", help="waive the Park check (standstill still enforced)")
    p.add_argument("--allow-running", action="store_true", help="waive the engine-off check (standstill + Park still enforced)")
    p.set_defaults(fn=cmd_xdrive)

    a = ap.parse_args(argv)
    return a.fn(a)


if __name__ == "__main__":
    raise SystemExit(main())
