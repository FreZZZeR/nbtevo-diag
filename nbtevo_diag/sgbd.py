"""Read an EDIABAS SGBD (`.prg`) and pull its UDS-22 parameter catalog.

This is the *catalog builder* the signals.py docstring points at: instead of guessing a
DID's scale from a gasoline reference (the `bmw-candidate` templates), read the names and
scalings straight out of BMW's own ECU description file. A `.prg` holds, XOR-0xF7 obscured,
a directory of tables; `SG_FUNKTIONEN` lists each job (a DID + its service + a result frame),
and the frame rows carry RESULTNAME / DATENTYP / MASKE / MUL / DIV / ADD / EINHEIT. That maps
one-to-one onto a `signals.Signal` + `Dec(width, num, den, offset, signed)`.

Format (derived first-hand from a real NBT EVO SGBD, not transcribed):
  - bytes[0:15] == b"@EDIABAS OBJECT"; the whole body is `byte ^ 0xF7`.
  - A header dword slot (plaintext) points at the table directory. The directory itself and
    every table body live in the *decrypted* stream.
  - directory: u32 count, then `count` entries of 64-byte name + u32 start,_,ncols,nrows.
  - table body at `start`: `ncols` cp1252 NUL-terminated column names, then `nrows` rows of
    `ncols` NUL-terminated cells.

What this does NOT do (YAGNI until a file needs it): DE->xx name translation, DTC/text
tables, T_GRTB address mapping (the DDE address 0x12 is already known), or the "read from
task code" heuristic. It reads tables and extracts UDS-22 scalar/bitfield parameters.

A value is still just a *candidate* after extraction: BMW's scale is right, but which car
exposes which DID — and sanity of the reading — is confirmed on the car, same as every other
signal (`verified=False` until then). A wrong-but-plausible number is worse than "n/a".

CLI:  nbtevo-diag sgbd FILE.prg [--tables | --params | --sig]
"""
from __future__ import annotations

import re
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional

MAGIC = b"@EDIABAS OBJECT"
XOR_KEY = 0xF7

# EDIABAS DATENTYP -> (code, width_bytes, signed). width 0 = "to end of response".
_INT_TYPES = {
    "unsigned char": ("U8", 1, False), "char": ("S8", 1, True), "signed char": ("S8", 1, True),
    "unsigned int": ("U16", 2, False), "unsigned integer": ("U16", 2, False),
    "int": ("S16", 2, True), "integer": ("S16", 2, True),
    "signed int": ("S16", 2, True), "signed integer": ("S16", 2, True),
    "unsigned long": ("U32", 4, False), "long": ("S32", 4, True), "signed long": ("S32", 4, True),
    "float": ("F32", 4, False), "motorola float": ("F32", 4, False), "intel float": ("F32", 4, False),
    "double": ("F64", 8, False), "motorola double": ("F64", 8, False), "intel double": ("F64", 8, False),
}


@dataclass
class Param:
    """One readable value: a DID + where/how to decode it. `order`='BE' matches signals.Dec
    (big-endian); `mask`!=0 means the field shares a byte (signals.Dec has no mask yet, so a
    masked param is data for a human/codegen, not a drop-in Dec)."""
    did: str            # 4 hex chars, e.g. "402B"
    result: str         # RESULTNAME, e.g. "STAT_RAILDRUCK_WERT"
    typ: str            # U8/S16/U32/F32/STRn/HEXn
    width: int          # bytes (0 = to end)
    pos: int            # byte offset within the 22-reply payload
    mask: int           # 0, or bitmask within the field
    num: float          # MUL
    den: float          # DIV (>=1)
    offset: float       # ADD
    unit: str
    order: str          # 'BE' or 'LE'
    info: str = ""

    @property
    def signed(self) -> bool:
        return self.typ.startswith("S")

    @property
    def is_scalar(self) -> bool:
        return self.typ[0] in "US" and self.width in (1, 2, 4) and not self.mask and self.order == "BE"


# --------------------------------------------------------------------------- decrypt + tables
def _plain(raw: bytes) -> bytes:
    return bytes(b ^ XOR_KEY for b in raw)


def _cstr(plain: bytes, off: int, mx: int = 4096):
    end = plain.find(b"\x00", off, off + mx)
    if end == -1:
        end = min(len(plain), off + mx)
    return plain[off:end].decode("cp1252", "replace").strip(), end + 1


def read_tables(path) -> Dict[str, List[dict]]:
    """{TABLE_NAME: [row dicts keyed by UPPERCASE column]}. Empty dict if not an SGBD."""
    raw = Path(path).read_bytes()
    if len(raw) < 0xA0 or not raw.startswith(MAGIC):
        return {}
    plain = _plain(raw)
    size = len(raw)
    # Header dword slots hold a *plaintext* pointer to the directory; the directory + bodies
    # are in the decrypted stream. The slot index varies between SGBDs, so scan and validate.
    for off in range(0x6C, 0x9C, 4):
        ptr = struct.unpack_from("<I", raw, off)[0]
        if not (0x90 <= ptr < size - 8):
            continue
        count = struct.unpack_from("<I", plain, ptr)[0]
        if not (0 < count <= 4000) or ptr + 4 + count * 80 > size:
            continue
        first = plain[ptr + 4:ptr + 4 + 64].split(b"\x00", 1)[0]
        if not first or not all(32 <= b <= 126 for b in first):
            continue
        tables = _walk_dir(plain, ptr, count, size)
        if tables:
            return tables
    return {}


def _walk_dir(plain: bytes, ptr: int, count: int, size: int) -> Dict[str, List[dict]]:
    tables: Dict[str, List[dict]] = {}
    pos = ptr + 4
    for _ in range(count):
        if pos + 80 > size:
            break
        name = plain[pos:pos + 64].split(b"\x00", 1)[0].decode("cp1252", "replace").strip().upper()
        start, _end, ncols, nrows = struct.unpack_from("<IIII", plain, pos + 64)
        pos += 80
        if not name or not (0x90 <= start < size) or ncols > 300 or nrows > 500000:
            continue
        tables[name] = _read_body(plain, start, ncols, nrows, size)
    return tables


def _read_body(plain, start, ncols, nrows, size) -> List[dict]:
    sp = start
    cols = []
    for _ in range(ncols):
        c, sp = _cstr(plain, sp)
        cols.append(c.upper())
    rows = []
    for _ in range(nrows):
        if sp >= size:
            break
        vals = []
        for _ in range(ncols):
            v, sp = _cstr(plain, sp)
            vals.append(v)
        rows.append(dict(zip(cols, vals)))
    return rows


# --------------------------------------------------------------------------- extraction
def _num(text, default: float) -> float:
    t = (text or "").strip()
    if t in ("", "-"):
        return default
    try:
        return float(int(t, 16)) if t.lower().startswith("0x") else float(t)
    except ValueError:
        return default


def _parse_type(datentyp: str):
    """(code, width, signed) or None. STRn/HEXn -> ('STRn'/'HEXn', n, False)."""
    d = (datentyp or "").strip().lower()
    if d in _INT_TYPES:
        return _INT_TYPES[d]
    m = re.fullmatch(r"(string|data)\s*(?:\[\s*(\d+)\s*\])?", d)
    if m:
        n = int(m.group(2)) if m.group(2) else 0
        return (("STR" if m.group(1) == "string" else "HEX") + str(n), n, False)
    return None


def _order(lh: str, datentyp: str) -> str:
    d = (datentyp or "").lower()
    if d.startswith("intel"):
        return "LE"
    if d.startswith("motorola"):
        return "BE"
    return "LE" if (lh or "").strip().lower() == "low" else "BE"


def _trailing_zeros(mask: int) -> int:
    return (mask & -mask).bit_length() - 1 if mask else 0


def _mk(did, row, code, width, signed, pos, mask, order) -> Param:
    return Param(
        did=did, result=(row.get("RESULTNAME") or "-").strip(),
        typ=code, width=width, pos=pos, mask=mask,
        num=_num(row.get("MUL"), 1.0), den=_num(row.get("DIV"), 1.0) or 1.0,
        offset=_num(row.get("ADD"), 0.0), unit=(row.get("EINHEIT") or "").strip(),
        order=order, info=(row.get("INFO") or row.get("BESCHREIBUNG") or "").strip()[:200],
    )


def _frame(tables, did, rows) -> Optional[List[Param]]:
    """Walk a result-frame table into positioned params. None if a field can't be sized
    (positions after it would drift — skip the whole frame rather than emit wrong offsets).
    ponytail: BITFIELD sub-tables aren't recursed into; a frame that uses one is skipped,
    not guessed. Add the recursion when a real SGBD needs those params."""
    out: List[Param] = []
    pos = 0
    group_type = None          # datentyp of the byte a run of masks shares
    group_mask = 0
    for r in rows:
        dt = (r.get("DATENTYP") or "").strip()
        mask = int(_num(r.get("MASKE"), 0))
        if dt.upper() == "BITFIELD":
            return None
        t = _parse_type(dt)
        if not t:
            return None
        code, width, signed = t
        order = _order(r.get("L/H"), dt)
        if mask:
            # another mask, same type, non-overlapping bits => same byte as the previous
            if group_type == dt and not (group_mask & mask):
                group_mask |= mask
            else:
                if group_type:
                    pos += _parse_type(group_type)[1]
                group_type, group_mask = dt, mask
            out.append(_mk(did, r, code, width, signed, pos, mask, order))
            continue
        if group_type:
            pos += _parse_type(group_type)[1]
            group_type = None
            group_mask = 0
        out.append(_mk(did, r, code, width, signed, pos, 0, order))
        if width == 0:
            break                 # "to end" is only valid as the last field
        pos += width
    return out


def params(tables: Dict[str, List[dict]]) -> List[Param]:
    """Every UDS-22 (ReadDataByIdentifier) readable value in the SGBD."""
    out: List[Param] = []
    skipped = 0
    for r in tables.get("SG_FUNKTIONEN", []):
        services = [s.strip().upper() for s in (r.get("SERVICE") or "").split(";")]
        if "22" not in services:
            continue
        did = (r.get("ID") or "").strip().upper().replace("0X", "")
        if not re.fullmatch(r"[0-9A-F]{4}", did):
            continue
        res = (r.get("RES_TABELLE") or "-").strip().upper()
        dt = (r.get("DATENTYP") or "-").strip()
        if res not in ("", "-"):
            frame = _frame(tables, did, tables.get(res, []))
            if frame:
                out.extend(frame)
            else:
                skipped += 1
        else:
            t = _parse_type(dt)
            if t:
                out.append(_mk(did, r, t[0], t[1], t[2], 0, 0, _order(r.get("L/H"), dt)))
            else:
                skipped += 1
    params.skipped = skipped      # type: ignore[attr-defined]
    return out


def to_sig(p: Param) -> str:
    """A signals.py `_sig(...)` candidate line (confidence 'bmw-candidate', unverified)."""
    key = re.sub(r"[^a-z0-9]+", "_", re.sub(r"^STAT_", "", p.result).lower()).strip("_")[:28] or f"did_{p.did.lower()}"
    if p.is_scalar:
        bits = [f"Dec({p.width}"]
        if p.num != 1:
            bits.append(f"num={p.num:g}")
        if p.den != 1:
            bits.append(f"den={p.den:g}")
        if p.offset:
            bits.append(f"offset={p.offset:g}")
        if p.signed:
            bits.append("signed=True")
        dec = ", ".join(bits) + ")"
    else:
        dec = "None"   # bitfield/string/LE/float: needs a hand-written decode
    note = f"SGBD {p.typ}"
    if p.mask:
        note += f" mask=0x{p.mask:X}>>{_trailing_zeros(p.mask)}"
    if p.order == "LE":
        note += " little-endian"
    return (f'_sig("{key}", "{p.result}", DME, UDS, 0x{p.did}, {dec}, '
            f'"{p.unit}", "misc", "bmw-candidate", note="{note}; validate on car"),')


# --------------------------------------------------------------------------- CLI
def run(path: str, mode: str) -> int:
    tables = read_tables(path)
    if not tables:
        print("not an EDIABAS SGBD (no @EDIABAS OBJECT header / directory):", path)
        return 1
    if mode == "tables":
        for name, rows in tables.items():
            cols = list(rows[0]) if rows else []
            print("  %-34s rows=%-6d cols=%s" % (name, len(rows), ",".join(cols)))
        print("\n%d tables. SG_FUNKTIONEN %s." % (
            len(tables), "present" if "SG_FUNKTIONEN" in tables else "ABSENT (no UDS-22 parameter catalog here)"))
        return 0
    ps = params(tables)
    if not ps:
        print("no UDS-22 parameters (SG_FUNKTIONEN %s)" % (
            "absent" if "SG_FUNKTIONEN" not in tables else "present but no readable DIDs"))
        return 1
    if mode == "sig":
        for p in ps:
            print(to_sig(p))
    else:
        for p in ps:
            sc = ("*%g" % p.num if p.num != 1 else "") + ("/%g" % p.den if p.den != 1 else "") \
                 + ("%+g" % p.offset if p.offset else "")
            m = " &0x%X" % p.mask if p.mask else ""
            print("  %s %-34s %-5s @%-2d%s %-8s %s" % (p.did, p.result, p.typ, p.pos, m, sc, p.unit))
    print("\n%d params from %d DIDs  (skipped frames: %d)" % (
        len(ps), len({p.did for p in ps}), getattr(params, "skipped", 0)))
    return 0


def _demo() -> bool:
    """Pure self-check: no file needed. Synthesises a tiny SGBD table set and asserts the
    SG_FUNKTIONEN walk, type->Dec mapping, position accounting and mask grouping."""
    tables = {
        "SG_FUNKTIONEN": [
            {"ID": "0x402B", "SERVICE": "22", "RES_TABELLE": "RT_RAIL", "DATENTYP": "-"},
            {"ID": "4A49", "SERVICE": "22", "RES_TABELLE": "-", "DATENTYP": "signed int",
             "MUL": "1", "DIV": "10", "ADD": "0", "EINHEIT": "deg", "RESULTNAME": "STAT_TIMING"},
            {"ID": "5F00", "SERVICE": "2E", "RES_TABELLE": "-", "DATENTYP": "unsigned char"},  # write, skip
            {"ID": "4FED", "SERVICE": "22", "RES_TABELLE": "RT_BITS", "DATENTYP": "-"},
        ],
        "RT_RAIL": [
            {"RESULTNAME": "STAT_RAILDRUCK_WERT", "DATENTYP": "unsigned int", "MASKE": "0",
             "MUL": "1", "DIV": "10", "ADD": "0", "EINHEIT": "bar", "L/H": "high"},
            {"RESULTNAME": "STAT_RAILDRUCK_SOLL", "DATENTYP": "unsigned int", "MASKE": "0",
             "MUL": "1", "DIV": "10", "ADD": "0", "EINHEIT": "bar", "L/H": "high"},
        ],
        "RT_BITS": [
            {"RESULTNAME": "FLAG_A", "DATENTYP": "unsigned char", "MASKE": "0x0F", "MUL": "1", "DIV": "1", "ADD": "0"},
            {"RESULTNAME": "FLAG_B", "DATENTYP": "unsigned char", "MASKE": "0xF0", "MUL": "1", "DIV": "1", "ADD": "0"},
            {"RESULTNAME": "TAIL", "DATENTYP": "unsigned char", "MASKE": "0", "MUL": "1", "DIV": "1", "ADD": "0"},
        ],
    }
    ps = params(tables)
    by = {(p.did, p.result): p for p in ps}
    # inline scalar
    t = by[("4A49", "STAT_TIMING")]
    assert t.typ == "S16" and t.width == 2 and t.signed and t.den == 10 and t.pos == 0
    # frame: two u16 at positions 0 and 2
    assert by[("402B", "STAT_RAILDRUCK_WERT")].pos == 0
    assert by[("402B", "STAT_RAILDRUCK_SOLL")].pos == 2
    # write service skipped
    assert not any(p.did == "5F00" for p in ps)
    # mask grouping: FLAG_A and FLAG_B share byte 0; TAIL falls at byte 1
    assert by[("4FED", "FLAG_A")].pos == 0 and by[("4FED", "FLAG_A")].mask == 0x0F
    assert by[("4FED", "FLAG_B")].pos == 0 and by[("4FED", "FLAG_B")].mask == 0xF0
    assert by[("4FED", "TAIL")].pos == 1
    assert _trailing_zeros(0xF0) == 4
    # to_sig emits a Dec for a clean scalar, None for a masked field
    assert "Dec(2, den=10" in to_sig(t)
    assert to_sig(by[("4FED", "FLAG_A")]).count("None") == 1
    return True


if __name__ == "__main__":
    assert _demo()
    print("sgbd self-check OK")
