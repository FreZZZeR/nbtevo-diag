"""Offline deploy + 6801-lease tests against a fake head unit — no car, no SSH.

FakeRemote simulates the unit's shell: an in-memory filesystem + process list, answering the
handful of atomic commands deploy issues. Proves the install order, the first-backup rule,
all-or-nothing rollback, the collector lease (stop-before / restart-after), the VIN consent
gate, and dry-run writing nothing — all without paramiko or a car.
"""
import os
import shlex
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from nbtevo_diag import deploy  # noqa: E402
from nbtevo_diag.deploy import Artifact, DeployError, collectors_paused  # noqa: E402

_QUIET = lambda *a, **k: None  # noqa: E731


class FakeRemote(deploy.Remote):
    """In-memory unit: files {path: bytes} + live process names; records every command."""
    def __init__(self, files=None, procs=()):
        self.files = dict(files or {})
        self.procs = list(procs)
        self.cmds = []

    def run(self, cmd):
        self.cmds.append(cmd)
        p = shlex.split(cmd)
        if cmd.startswith("pidin"):
            return 0, "\n".join(self.procs), ""
        if p[0] == "slay":
            name = p[-1]
            self.procs = [x for x in self.procs if name not in x]
            return 0, "", ""
        if p[0] == "mount":
            return 0, "", ""
        if p[0] == "[" and p[1] == "-f":
            return (0 if p[2] in self.files else 1), "", ""
        if p[0] == "cp":
            if p[1] in self.files:
                self.files[p[2]] = self.files[p[1]]
                return 0, "", ""
            return 1, "", "no file"
        if p[0] == "wc":                       # wc -c < path
            path = p[-1]
            return (0, "%d\n" % len(self.files[path]), "") if path in self.files else (1, "", "no file")
        if p[0] == "mv":
            if p[1] in self.files:
                self.files[p[2]] = self.files.pop(p[1])
                return 0, "", ""
            return 1, "", "no file"
        if p[0] == "chmod":
            return 0, "", ""
        if p[0] == "rm":
            for path in [x for x in p[1:] if not x.startswith("-")]:
                self.files.pop(path, None)
            return 0, "", ""
        return 0, "", ""                        # restart cmd, start_cmd, etc.

    def put(self, data, path):
        self.cmds.append("PUT %s" % path)
        self.files[path] = bytes(data)


def _artifact(content, remote, mode="755"):
    f = tempfile.NamedTemporaryFile(delete=False)
    f.write(content)
    f.close()
    return Artifact(f.name, remote, mode)


def _set_vin(on):
    if on:
        os.environ["NBTEVO_OWN_VIN"] = "TESTVIN0DEPLOY000"
    else:
        os.environ.pop("NBTEVO_OWN_VIN", None)


def test_dry_run_writes_nothing():
    fake = FakeRemote()
    ok = deploy.deploy("h", [_artifact(b"SO", "/p/app.so")], commit=False, remote=fake, log=_QUIET)
    assert ok is False
    assert fake.files == {}
    assert not any(c.startswith("PUT") for c in fake.cmds)


def test_commit_installs_new_file():
    _set_vin(True)
    fake = FakeRemote()
    ok = deploy.deploy("h", [_artifact(b"SO1", "/p/app.so")], commit=True, remote=fake, log=_QUIET)
    assert ok is True
    assert fake.files["/p/app.so"] == b"SO1"
    assert "/p/app.so.orig" not in fake.files        # was new -> no backup


def test_commit_backs_up_existing():
    _set_vin(True)
    fake = FakeRemote(files={"/p/app.so": b"OLD"})
    deploy.deploy("h", [_artifact(b"NEW", "/p/app.so")], commit=True, remote=fake, log=_QUIET)
    assert fake.files["/p/app.so"] == b"NEW"
    assert fake.files["/p/app.so.orig"] == b"OLD"     # first backup kept


def test_rollback_is_all_or_nothing():
    _set_vin(True)
    fake = FakeRemote(files={"/p/a.so": b"A0"})
    arts = [_artifact(b"A1", "/p/a.so"), _artifact(b"", "/p/b.so")]  # second file empty -> fails
    try:
        deploy.deploy("h", arts, commit=True, remote=fake, log=_QUIET)
    except DeployError:
        pass
    else:
        raise AssertionError("an empty artifact should abort the deploy")
    assert fake.files["/p/a.so"] == b"A0"             # first file rolled back to its .orig
    assert "/p/b.so" not in fake.files                # second never landed


def test_vin_consent_gate():
    _set_vin(False)
    fake = FakeRemote()
    try:
        deploy.deploy("h", [_artifact(b"SO", "/p/app.so")], commit=True, remote=fake, log=_QUIET)
    except DeployError:
        pass
    else:
        raise AssertionError("commit without NBTEVO_OWN_VIN should refuse")
    assert fake.files == {}


def test_lease_pauses_and_restarts():
    fake = FakeRemote(procs=["nbtevo-collector", "qnx-io"])
    with collectors_paused(fake, ["nbtevo-collector"], start_cmd="start-collectors", log=_QUIET):
        assert "nbtevo-collector" not in fake.procs   # stopped for the body
    assert "start-collectors" in fake.cmds            # restarted after


def test_lease_noop_when_not_running():
    fake = FakeRemote(procs=["qnx-io"])
    with collectors_paused(fake, ["nbtevo-collector"], start_cmd="start-collectors", log=_QUIET):
        pass
    assert not any(c.startswith("slay") for c in fake.cmds)       # nothing to stop
    assert "start-collectors" not in fake.cmds                    # nothing to restart


def test_commit_stops_collectors_before_upload():
    _set_vin(True)
    fake = FakeRemote(procs=["col"])
    deploy.deploy("h", [_artifact(b"SO", "/p/app.so")], collectors=["col"],
                  start_collectors_cmd="go", commit=True, remote=fake, log=_QUIET)
    slay = next(i for i, c in enumerate(fake.cmds) if c.startswith("slay"))
    put = next(i for i, c in enumerate(fake.cmds) if c.startswith("PUT"))
    start = next(i for i, c in enumerate(fake.cmds) if c == "go")
    assert slay < put < start                         # free 6801, transfer, then restart


def test_revert_restores_orig():
    _set_vin(True)
    fake = FakeRemote(files={"/p/app.so": b"NEW", "/p/app.so.orig": b"OLD"})
    deploy.revert("h", [Artifact("x", "/p/app.so")], commit=True, remote=fake, log=_QUIET)
    assert fake.files["/p/app.so"] == b"OLD"
    assert "/p/app.so.orig" not in fake.files         # backup consumed by the restore


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print("ok", name)
    print("all tests passed")
