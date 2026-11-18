# nbtevo-diag

On-head-unit diagnostics for the **BMW NBT EVO** (built for an **F30 335d**, N57 diesel, iDrive6):
fault codes across modules and live engine/vehicle sensors, shown **in iDrive, with no OBD
adapter**. The head unit already sits on the car's diagnostic network — it reaches every module
through the FEM gateway over HSFZ/UDS, the same plane evocode speaks. No dongle, no phone.

> Owner's car only, over a local connection. Reads are safe; the one write (clear DTCs) is gated
> behind `--commit`. It never touches coding, FSC or firmware.

## Architecture (3 layers)

1. **HMI plugin** — an iDrive menu with **Ошибки** (DTCs) and **Датчики** (gauges). Native C++/QNX;
   plugin ABI from the NBT-EVO toolkit SDK / NBT-AAD. Visual target: the BoostDeck/ShiftDeck decks.
2. **Collector** — a C worker on the head unit, polls the internal gateway (HSFZ 6801 → UDS) and
   feeds the plugin. Template: ID6_apps `collector.c`.
3. **Data engine** — the DID map, decode math and DTC logic. **This repo, in Python, is that engine**
   (Phase 0), built on evocode's transport so it can be developed and the signals *discovered* from a
   laptop over ENET before any native code exists.

**Phase 0 (now):** the Python engine + CLI — read DTCs, read the signals we already know, and
**probe/validate the N57's real DIDs** → produce the per-car map. Zero brick risk, no toolchain.
**Phase 1 (later, needs QNX SDP 6.5 + ARM GCC 4.4.2):** port the engine into the native collector +
HMI plugin so it runs from iDrive.

## Transport facts (from the toolkit's RE)

- One socket to the **FEM gateway `0x10`** reaches every module by the **UDS target byte** (HSFZ, not
  DoIP; no routing-activation). DDE `0x12`, EGS `0x18`, head unit `0x63`.
- Module list in one shot: functional TesterPresent `3E 00` → `0xDF`, every awake ECU answers `7E`.
- **DTCs:** `19 02 FF` per module, default session, no SecurityAccess. Clear: `14 FF FF FF`.
- Live values: standard OBD mode-01 PIDs, and BMW-specific DIDs via `22`.

## Install & use (Phase 0, from a dev machine on ENET)

```bash
pip install nbtevo-diag                 # core: diagnostics over ENET (pulls evocode)
pip install "nbtevo-diag[deploy]"       # + evocode[ssh]/paramiko, to ship the app to the head unit
# or from a checkout:  pip install -e .

nbtevo-diag catalog                          # the signal catalog (offline, no car)
nbtevo-diag modules --host 169.254.x.x       # enumerate awake ECUs
nbtevo-diag dtc     --host 169.254.x.x       # read fault memory (all modules)
nbtevo-diag live    --host 169.254.x.x       # coolant/oil/boost/power/... live
nbtevo-diag scan    --host 169.254.x.x --ecu 0x12 --hunt   # discover N57 DIDs
```

`--host` is the FEM gateway IP (ENET ident broadcast / evocode discovery).

## The signal catalog — and what people actually want

`nbtevo_diag/signals.py` is the single source of truth. Each signal carries a **confidence**:

- **✓ standard** — SAE J1979 OBD mode-01. Works on the diesel, standardised formulas, verifiable at
  once. This already covers a lot of the "dashboard + OBD" wishlist: **coolant, oil temp, intake air,
  ambient, RPM, speed, MAP/boost, MAF, load, throttle, pedal, timing, fuel rate & level, module
  voltage, commanded EGR, run time**, and — the "sport displays" numbers — **actual torque %,
  reference torque (Nm), and power** (torque × rpm).
- **ƒ computed** — **boost** (MAP − baro, bar/psi), **torque** (Nm), **power** (kW/PS).
- **? bmw-candidate** — BMW DIDs from the B58/340i **gasoline** reference (ID6_apps). A *template*,
  not valid for the N57 until proven: boost/ambient `4205`/`4201`, rail `56D7`, oil-press `586F`,
  coolant `4300`, trans-temp `DA12` (EGS). Note: rail scaling is gas-GDI — wrong for common-rail.
- **· probe** — wanted but address/decode unknown → shown as **n/a** until discovered: **rail
  pressure, oil pressure** (may be switch-only on the N57), **turbo speed, VGT/wastegate, injection
  quantity, EGT, DPF soot load / regen / differential pressure, AdBlue (SCR) level**, and —
  different modules — **brake pad wear, TPMS, CBS service items**.

Commonly wanted beyond your list (street/track/daily), already grouped in the catalog: **DPF soot &
regen and AdBlue** (diesel-critical), **transmission temp** (track), **EGT**, **battery voltage**,
**fuel rate/range**, **tyre pressures**, **CBS service remaining**. Computed extras worth adding in
Phase 1 from speed/time: **0–100 km/h, ¼-mile, lap timer, G-meter**.

Discovered per-car truth goes in `maps/<car>.json` (gitignored) — the catalog holds method +
candidates; a car's verified DIDs/scalings live in its map. A value only shows once validated against
an independent reference (ISTA / cluster / a logger); otherwise it stays **n/a** rather than lie.

## Testing without the car

`python -m pytest tests/` (or `python tests/test_signals.py`) runs the pure checks — catalog
integrity and decode math — with no car and no evocode. **IDEmu can't test this layer**: the emulator
doesn't expose the HSFZ diagnostic endpoint (6801), so it's only useful later for rendering the HMI
plugin, not for the data engine. The data engine is validated on the car (or a mock gateway).

## xDrive control — actuation, with a software fuse

Beyond read-only diagnostics, one **actuation** is included: xDelete-style **temporary rear-wheel
drive** (ported from ID6_apps). The DSC (`0x29`) commands the transfer case via `2E DB E1 <0=RWD |
1=AWD>`, after a variant + capability check, with a mode read-back.

The reference refuses the write unless the car is stationary, in Park **and the engine is off** —
and in BM3 form that last part is just a label the human is trusted to honour. Here every
condition is a **code-enforced fuse** (`control.py`), engine-off included:

- engaging RWD requires **standstill (speed = 0)** — a floor that is **not** bypassable;
- Park is required by default (`--no-park-check` waives it; standstill still holds);
- **engine-off is the default**, enforced in code — not dropped, not a label. Waive it
  **consciously** with `--allow-running` (standstill + Park still hold). It stays the default
  because the write is latch-like (the `0x0F2510` DSC only restores AWD on an ignition cycle) and
  the module tracks RWD as a degraded state — engine-off is the quiescent, zero-driveline-torque
  moment the controller expects;
- **dry-run by default**; a real write needs `--commit`, which is additionally **VIN-gated**
  (`NBTEVO_OWN_VIN`); the result is read back to confirm; AWD restores on an ignition cycle.

```bash
nbtevo-diag xdrive --host IP status          # variant / mode / speed / park / engine
nbtevo-diag xdrive --host IP rwd             # dry-run: the plan + the interlock result
NBTEVO_OWN_VIN=<your VIN> nbtevo-diag xdrive --host IP rwd --commit                  # engage RWD (stopped, Park, engine off)
NBTEVO_OWN_VIN=<your VIN> nbtevo-diag xdrive --host IP rwd --commit --allow-running  # ...with the engine running, consciously
nbtevo-diag xdrive --host IP awd --commit    # restore AWD
```

> ⚠ Disabling the front axle changes how the car behaves (traction, DSC/ABS). Track / private land
> only, never on a public road, on your own car, at your own risk. The DSC variant + DIDs come from a
> 340i (ID6_apps) — validate them on the 335d's DSC before a live write.

## Deploy — ship the native app to the head unit

The on-car layer ([nbtevo-id6-apps](https://github.com/FreZZZeR/nbtevo-id6-apps): collector +
HMI plugin) is **delivered by this Python layer**, not by forking anyone's installer. `deploy.py`
does two things, both reversible and dry-run by default:

- **6801-lease coordination.** The on-head-unit collectors hold a private loopback lease on the
  single diagnostic gateway (TCP 6801) that a laptop can't see — so running evocode/diag while a
  collector is live means mutual timeouts. `deploy` (and any diagnostic session) **pauses the
  collectors** for its duration and restarts them after (`collectors_paused`).
- **Safe multi-file install** over the unit's dev SSH, reusing evocode's `fw` choreography (keep a
  first `.orig` backup → upload a same-partition temp → verify the on-unit byte count → atomic
  rename-over → chmod), and **all-or-nothing**: any failure rolls the whole set back.

```bash
# dry-run (writes nothing): show the plan + the lease steps
nbtevo-diag deploy --host IP  build/libNbtGauges.so:/var/opt/hmi/data/ID5/Plugins/Gauges/libNbtGauges.so \
                              --collector nbtevo-collector --restart appreset
# perform it (VIN-gated), pausing the named collector for the transfer
NBTEVO_OWN_VIN=<your VIN> nbtevo-diag deploy --host IP ... --collector nbtevo-collector \
                              --start-collectors "/var/opt/car/bin/nbtevo-collector &" --commit
nbtevo-diag deploy-revert --host IP /var/opt/hmi/data/ID5/Plugins/Gauges/libNbtGauges.so --commit
```

> The write path is **not yet exercised on a live unit** (it is host-tested against a fake unit).
> The collector process name(s) are supplied by you — they are the on-car app's, not hard-coded.
> `--restart appreset` restarts just the HMI (PID-verified — some heads' appreset returns 0 without
> resetting); for a full stock re-read with no power-cycle use `evocode reset` (UDS 11 01).

## Credits

Built on the diagnostic plane of **evocode** (transport/UDS). Transport RE, module addressing and the
plugin SDK from **[vandelayautollc/nbt-evo-toolkit](https://github.com/vandelayautollc/nbt-evo-toolkit)**
(MIT). Live-signal collector pattern and B58 candidate DIDs from
**[inotbarnacle/ID6_apps_prerelease](https://github.com/inotbarnacle/ID6_apps_prerelease)**. Plugin
ABI lineage: **NBT-AAD**. License: Apache-2.0.

## Development guardrail

Committed git hooks block publishing private data and run the tests before every push.
Activate once per clone: `git config core.hooksPath hooks` (details in `hooks/README.md`).
