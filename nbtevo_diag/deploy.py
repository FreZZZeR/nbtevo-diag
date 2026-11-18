"""Deliver the native app to the head unit — safe, reversible, lease-aware.

The on-car layer (nbtevo-id6-apps: collector + HMI plugin) is DELIVERED by this Python layer,
**not** by forking ID6_apps' Go installer. Two jobs live here:

1. **6801-lease coordination.** The on-HU collectors arbitrate the single diagnostic gateway
   (TCP 6801) through a private loopback lease the laptop can't see. Running evocode /
   nbtevo-diag diagnostics while a collector is live = mutual timeouts. So a laptop session
   PAUSES the collectors (`collectors_paused`) for its duration, then restarts them.

2. **Safe multi-file install.** Ship the collector binary + the HMI plugin `.so` over the unit's
   dev SSH, each with evocode.fw's choreography (keep a first `.orig` backup -> upload a temp on
   the same partition -> verify the on-unit byte-count -> atomic rename-over -> chmod), and
   **all-or-nothing**: if any file fails, roll the whole set back.

Dry-run by default; a real write needs ``commit=True`` and the ``NBTEVO_OWN_VIN`` consent gate.
SSH transport is reused from evocode.fw (the legacy-KEX handling the old QNX sshd needs lives
there); the deploy logic talks to a small ``Remote`` seam so it is host-testable with a fake.
"""
from __future__ import annotations

import os
import shlex
import time
from contextlib import contextmanager
from dataclasses import dataclass
from typing import Callable, List, Optional, Sequence, Tuple

# evocode provides the SSH transport. Prefer the installed package; fall back to a dev checkout
# (same convenience as transport.py). fw.py imports paramiko lazily, so this import is cheap and
# works even without the ssh extra — only SshRemote.connect/put/get actually need paramiko.
try:
    from evocode import fw
except Exception:  # pragma: no cover - dev convenience
    import sys
    _ev = os.environ.get("EVOCODE_PATH") or os.path.expanduser("~/Documents/evocode")
    if os.path.isdir(_ev):
        sys.path.insert(0, _ev)
    from evocode import fw  # noqa: E402

_q = shlex.quote


class DeployError(Exception):
    pass


# --- the SSH seam: deploy logic talks to this; tests inject a fake ------------
class Remote:
    """Minimal remote-shell interface deploy needs: run a command, upload bytes.

    The real one (SshRemote) wraps evocode.fw over one connection; tests use an in-memory fake.
    """
    def run(self, cmd: str) -> Tuple[int, str, str]:
        raise NotImplementedError

    def put(self, data: bytes, path: str) -> None:
        raise NotImplementedError


class SshRemote(Remote):
    """A Remote over evocode.fw — one connection, QNX legacy ssh-rsa handled in fw.connect."""
    def __init__(self, cli):
        self._cli = cli

    @classmethod
    def connect(cls, host: str, **conn) -> "SshRemote":
        return cls(fw.connect(host, **conn))

    def run(self, cmd: str) -> Tuple[int, str, str]:
        return fw.run(self._cli, cmd)

    def put(self, data: bytes, path: str) -> None:
        fw.put(self._cli, data, path)

    def get(self, path: str) -> bytes:
        return fw.get(self._cli, path)

    def close(self) -> None:
        self._cli.close()


@dataclass
class Artifact:
    """One file to ship: a local build output -> an absolute path on the head unit."""
    local_path: str
    remote_path: str
    mode: str = "755"

    def data(self) -> bytes:
        with open(self.local_path, "rb") as f:
            return f.read()


# --- small command helpers (conditionals stay in Python; the unit sees atomic commands) ------
def _ok(rem: Remote, cmd: str) -> None:
    rc, o, e = rem.run(cmd)
    if rc != 0:
        raise DeployError("remote command failed (rc=%d): %s :: %s" % (rc, cmd, (e or o).strip()[:160]))


def _exists(rem: Remote, path: str) -> bool:
    rc, _o, _e = rem.run("[ -f %s ]" % _q(path))
    return rc == 0


def _size(rem: Remote, path: str) -> Optional[int]:
    rc, o, _e = rem.run("wc -c < %s" % _q(path))
    parts = o.split()
    return int(parts[0]) if (rc == 0 and parts and parts[0].isdigit()) else None


# --- 6801 lease: pause the on-HU collectors so the laptop owns the gateway -------------------
def collectors_running(rem: Remote, names: Sequence[str]) -> List[str]:
    """Which of ``names`` are live on the unit (QNX ``pidin``). Empty if pidin is unavailable."""
    rc, out, _e = rem.run("pidin -F '%n'")
    if rc != 0:
        return []
    live = out.split()
    return [n for n in names if any(n in p for p in live)]


def stop_collectors(rem: Remote, names: Sequence[str], log: Callable = print) -> None:
    for n in names:
        rem.run("slay -f %s" % _q(n))              # QNX: terminate by name
    still = collectors_running(rem, names)
    if still:
        raise DeployError("collectors still live after slay: %s" % ", ".join(still))
    log("  lease: collectors stopped, 6801 free (%s)" % ", ".join(names))


def start_collectors(rem: Remote, start_cmd: Optional[str], log: Callable = print) -> None:
    if start_cmd:
        rc, _o, e = rem.run(start_cmd)
        log("  lease: collectors restarted%s" % ("" if rc == 0 else "  -> rc=%d %s" % (rc, e.strip()[:120])))
    else:
        log("  lease: collectors NOT restarted here (auto-start on HMI/boot); pass start_cmd to re-launch")


@contextmanager
def collectors_paused(rem: Remote, names: Sequence[str], start_cmd: Optional[str] = None,
                      log: Callable = print):
    """Own the 6801 gateway for the body: stop the live collectors, always restart them after."""
    were = collectors_running(rem, names) if names else []
    if were:
        stop_collectors(rem, were, log)
    try:
        yield were
    finally:
        if were:
            start_collectors(rem, start_cmd, log)


# --- the safe per-file swap (on a shared Remote), with a rollback closure --------------------
def _install_one(rem: Remote, art: Artifact, log: Callable) -> Callable[[], None]:
    data = art.data()
    if not data:
        raise DeployError("refusing to install an empty file: %s" % art.local_path)
    r, tmp = art.remote_path, art.remote_path + ".new"
    had = _exists(rem, r)
    if had and not _exists(rem, r + ".orig"):
        _ok(rem, "cp %s %s.orig" % (_q(r), _q(r)))     # keep the FIRST backup only
    rem.put(data, tmp)                                  # upload to a same-partition temp
    got = _size(rem, tmp)                               # verify BEFORE the rename-over
    if got != len(data):
        rem.run("rm -f %s" % _q(tmp))
        raise DeployError("size mismatch on %s (sent %d, on-unit %s) — not replaced"
                          % (r, len(data), got))
    _ok(rem, "mv %s %s" % (_q(tmp), _q(r)))             # atomic same-fs rename -> new inode
    if art.mode:
        rem.run("chmod %s %s" % (_q(art.mode), _q(r)))
    log("  installed %s (%d bytes)%s" % (r, len(data), "" if had else "  [new]"))

    def rollback() -> None:
        if had and _exists(rem, r + ".orig"):
            rem.run("mv %s.orig %s" % (_q(r), _q(r)))   # restore the pre-deploy file
        elif not had:
            rem.run("rm -f %s" % _q(r))                 # it was new -> remove it
    return rollback


# --- HMI restart over SSH (gentle, HMI-only) -------------------------------------------------
# ``appreset`` restarts just the HMI app (not the OS), so the menu re-reads the Plugins dir
# without a full reboot. The tool lives at one of these paths; it is NOT boloreset/transport/flash
# (bootloader / service modes — never touch those). On some heads appreset returns 0 but does not
# actually reset (missing libsys_dsi_servicebroker); a real restart gives HmiMain3D a *new* PID, so
# we verify by PID. When appreset doesn't take, reboot the stock way instead: ``evocode reset``
# (UDS 11 01, a full re-read with no power-cycle).
_ONOFF_BINS = ("/opt/core/bin/OnOffDSICommander", "/opt/car/bin/OnOffDSICommander")


def _hmi_pid(rem: Remote) -> Optional[str]:
    """PID of HmiMain3D on the unit (QNX ``pidin``), or None if not found / pidin absent."""
    rc, out, _e = rem.run("pidin -F '%p %n'")
    if rc != 0:
        return None
    for line in out.splitlines():
        if "HmiMain3D" in line:
            tok = line.split()
            if tok and tok[0].isdigit():
                return tok[0]
    return None


def restart_hmi(rem: Remote, *, wait: int = 20, log: Callable = print) -> bool:
    """Restart only the HMI (``OnOffDSICommander appreset``), verified by a new HmiMain3D PID.

    Returns True only if HmiMain3D actually came back with a *different* PID (appreset can return 0
    yet not reset). Never touches boloreset/transport/flash. When it doesn't take, reboot the stock
    way: ``evocode reset`` (UDS 11 01).  ponytail: assumes QNX ``pidin``; no-op-safe if it's absent.
    """
    onoff = next((b for b in _ONOFF_BINS if _exists(rem, b)), None)
    if not onoff:
        _rc, out, _e = rem.run("command -v OnOffDSICommander")
        onoff = out.strip() or None
    if not onoff:
        log("  restart: no OnOffDSICommander on the unit — reboot with `evocode reset` (UDS 11 01)")
        return False
    before = _hmi_pid(rem)
    rem.run("%s appreset" % _q(onoff))
    time.sleep(wait)                                    # appreset returns 0 even when it didn't reset
    after = _hmi_pid(rem)
    ok = bool(after) and after != before
    log("  restart: appreset %s (HmiMain3D %s -> %s)"
        % ("OK" if ok else "did NOT take — reboot with `evocode reset`", before, after))
    return ok


def _do_restart(rem: Remote, restart: Optional[str], log: Callable) -> None:
    """Run the post-swap restart: the ``appreset`` sentinel -> restart_hmi, else a literal command."""
    if restart == "appreset":
        restart_hmi(rem, log=log)
    elif restart:
        rc, _o, e = rem.run(restart)
        log("  restart: %s%s" % (restart, "" if rc == 0 else "  -> rc=%d %s" % (rc, e.strip()[:120])))


def deploy(host: str, artifacts: Sequence[Artifact], *, mount: str = "/fs/sda0",
           restart: Optional[str] = None, collectors: Sequence[str] = (),
           start_collectors_cmd: Optional[str] = None, commit: bool = False,
           log: Callable = print, remote: Optional[Remote] = None, **conn) -> bool:
    """Deliver ``artifacts`` to the unit. Dry-run unless ``commit=True`` (+ NBTEVO_OWN_VIN).

    All-or-nothing: a failure part-way rolls every installed file back. Collectors are paused
    for the transfer (6801 lease) and restarted after. ``remote`` injects a Remote (tests);
    otherwise an SshRemote is opened to ``host``.
    """
    if not commit:
        log("DRY RUN — would deploy %d file(s) to %s (pass commit=True to perform):" % (len(artifacts), host))
        if collectors:
            log("  lease: pause collectors for the transfer, then restart: %s" % ", ".join(collectors))
        for a in artifacts:
            for step in fw.plan(a.remote_path, mount, a.remote_path + ".new", len(a.data()), restart=None):
                log("    %s: %s" % (os.path.basename(a.remote_path), step))
        if restart:
            log("  restart: %s" % ("OnOffDSICommander appreset (HMI-only, PID-verified)"
                                   if restart == "appreset" else restart))
        return False

    if not os.environ.get("NBTEVO_OWN_VIN"):
        raise DeployError("refusing to deploy without consent: set NBTEVO_OWN_VIN=<your VIN>")

    own = remote if remote is not None else SshRemote.connect(host, **conn)
    done: List[Callable[[], None]] = []
    try:
        with collectors_paused(own, collectors, start_collectors_cmd, log):
            _ok(own, "mount -uw %s" % _q(mount))
            try:
                for a in artifacts:
                    done.append(_install_one(own, a, log))
            except Exception as e:
                log("  !! %s — rolling back %d installed file(s)" % (e, len(done)))
                for rb in reversed(done):
                    try:
                        rb()
                    except Exception as re:  # noqa: BLE001 - keep rolling back the rest
                        log("    rollback error: %s" % re)
                own.run("mount -ur %s" % _q(mount))
                raise
            own.run("mount -ur %s" % _q(mount))
            _do_restart(own, restart, log)
        log("  *** deployed %d file(s); backups kept as *.orig. Watch the HMI come up. ***" % len(artifacts))
        return True
    finally:
        if remote is None and isinstance(own, SshRemote):
            own.close()


def revert(host: str, artifacts: Sequence[Artifact], *, mount: str = "/fs/sda0",
           restart: Optional[str] = None, collectors: Sequence[str] = (),
           start_collectors_cmd: Optional[str] = None, commit: bool = False,
           log: Callable = print, remote: Optional[Remote] = None, **conn) -> bool:
    """Restore each artifact's ``.orig`` backup (reverse order). Dry-run unless ``commit=True``."""
    if not commit:
        log("DRY RUN — would restore %d file(s) from *.orig (pass commit=True):" % len(artifacts))
        for a in artifacts:
            log("    %s <- %s.orig" % (a.remote_path, a.remote_path))
        return False

    own = remote if remote is not None else SshRemote.connect(host, **conn)
    try:
        with collectors_paused(own, collectors, start_collectors_cmd, log):
            _ok(own, "mount -uw %s" % _q(mount))
            for a in reversed(list(artifacts)):
                r = a.remote_path
                if _exists(own, r + ".orig"):
                    _ok(own, "mv %s.orig %s" % (_q(r), _q(r)))
                    log("  reverted %s from .orig" % r)
                else:
                    log("  no %s.orig on the unit — skipped" % r)
            own.run("mount -ur %s" % _q(mount))
            _do_restart(own, restart, log)
        return True
    finally:
        if remote is None and isinstance(own, SshRemote):
            own.close()
