# X2C-Scope from Python (`x2c_diag.py`)

Read, plot and **write** the firmware's global variables on a running target, without
MPLAB X. This is the working data-visualisation path for this project — the MPLAB X
X2C-Scope plugin fails with *"No symbols loaded"* on this device family (see
[Why not the MPLAB X plugin](#why-not-the-mplab-x-plugin)).

```bash
pip install pyx2cscope
python tools/x2c_diag.py watch
```

---

## Contents

- [How it works](#how-it-works)
- [The six subcommands](#the-six-subcommands)
- [Reading the script](#reading-the-script)
- [Recipes: logging your own variables](#recipes-logging-your-own-variables)
- [Recipes: changing behaviour at run time](#recipes-changing-behaviour-at-run-time)
- [Writing a custom script](#writing-a-custom-script)
- [Gotchas](#gotchas)
- [Why not the MPLAB X plugin](#why-not-the-mplab-x-plugin)

---

## How it works

Three pieces have to agree:

| Piece | Where | Role |
|---|---|---|
| `X2CScopeLib.X.a` + `diagnostics.c` | `project/x2cscope/` | On-target library. Owns a 4900-byte RAM buffer and answers LNet requests over UART1. |
| The production **ELF** | `project/pfc.X/dist/default/production/pfc.X.production.elf` | Carries the DWARF debug info that maps `pfcParam.iL` to a RAM address. |
| `pyx2cscope` | PC | Parses the ELF, then reads/writes those addresses over the serial link. |

The target never halts. `X2CScope_Update()` samples into the buffer from inside the ADC
ISR; `X2CScope_Communicate()` drains it from the main loop. Both are already wired up:

- `DiagnosticsInit()` — [`project/main.c`](../project/main.c), at boot
- `DiagnosticsStepIsr()` — [`project/pfc/pfc.c`](../project/pfc/pfc.c), in the ADC ISR
- `DiagnosticsStepMain()` — [`project/main.c`](../project/main.c), in `while(1)`

> **The ELF must match the binary on the target.** Addresses come from the ELF, not the
> chip. Reprogram after every rebuild, or you will read plausible values from the wrong
> addresses. This is the single most common way to waste an afternoon.

### Two ways to read data

**Polled (`watch`, `get`)** — the PC asks for one variable at a time. Simple, unlimited
duration, but the rate is set by USB round-trips (a few hundred Hz at best). Use it for
slow things: state, fault status, bus voltage, checking that a change took effect.

**Buffered (`scope`)** — the target fills its own RAM buffer at the ISR rate, then dumps
it. This is the only way to see switching-cycle detail. Depth is limited:

```
4900 bytes / (bytes per sample-set) = samples per channel
4 floats = 16 B/set -> ~306 sets -> 306 x 62.5 us = 19 ms
```

19 ms is just under one 50 Hz mains cycle, so for line-frequency envelopes use
`--factor 2` or more, or watch fewer/narrower channels (`int16` raws instead of floats
doubles the depth).

---

## The six subcommands

| Command | Hardware needed | What it does |
|---|---|---|
| `symbols [filter]` | no | Parse the ELF and list matching variables with type, size, address. |
| `probe` | board powered | Walk the serial link layer by layer and name the layer that failed. |
| `watch [vars...]` | running target | Poll variables, one line per interval. Ctrl-C to stop. |
| `scope [vars...]` | running target | Buffered capture at the ISR rate into CSV. |
| `get <var>` | running target | Read one value and exit. |
| `set <var> <value>` | running target | Write one value. |

```bash
# What can I even look at?
python tools/x2c_diag.py symbols                  # all 88 pfcParam members
python tools/x2c_diag.py symbols duty             # just the duty-related ones

# Is the link alive?
python tools/x2c_diag.py probe

# Live view (default set), or name your own
python tools/x2c_diag.py watch
python tools/x2c_diag.py watch pfcParam.state pfcParam.faultStatus --interval 0.2

# Buffered capture
python tools/x2c_diag.py scope --out ccm.csv --factor 4
python tools/x2c_diag.py scope --trigger pfcParam.dcmDetected --level 0.5 --out dcm_entry.csv

# One-offs
python tools/x2c_diag.py get pfcParam.pfcVoltage.vdc
python tools/x2c_diag.py set pfcParam.sampleCorrectionEnable 2
```

`--port COMn` overrides port auto-detection; `--baud` overrides 115200.

---

## Reading the script

The file is one module, ~330 lines, in four layers. To change behaviour you almost always
only touch the **first** layer.

### 1. Configuration constants (top of file)

```python
ELF = REPO / "project" / "pfc.X" / "dist" / "default" / "production" / "pfc.X.production.elf"
BAUD = 115200
TARGET_VID_PID = "04D8:00DD"     # MCP2221A on the PIM
SAMPLE_PERIOD_S = 62.5e-6        # 64 kHz ISR / PFC_DIAGNOSTICS_DECIMATION (4)
CHANNELS_DCM   = [...]           # default channels for `scope`
CHANNELS_WATCH = [...]           # default channels for `watch`
```

`SAMPLE_PERIOD_S` is derived, not measured. If you change `PFC_PWMFREQUENCY_HZ`
(`project/hal/pwm.h`) or `PFC_DIAGNOSTICS_DECIMATION` (`project/pfc/pfc_userparams.h`),
**update it here too** or every CSV time axis is silently wrong.

### 2. Connection helpers

- `find_port()` — matches the MCP2221A by USB VID:PID, because Windows renumbers the
  COM port on every replug into a different socket. Never hardcode `COM8`.
- `open_scope()` — returns a connected `X2CScope`. The constructor connects immediately
  and raises if the target does not answer.

### 3. One `cmd_*` function per subcommand

Each takes the parsed `args` and returns an exit code. They are independent — copy one as
the starting point for a new mode.

### 4. `main()` — argparse wiring

Each subcommand is registered in a four-line block. Port resolution happens once, after
parsing, and is skipped for `symbols` (the only hardware-free mode):

```python
if args.cmd != "symbols":
    args.port = find_port(args.port)
```

---

## Recipes: logging your own variables

### Log different variables, no code change

Pass them positionally. Any name from `symbols` works:

```bash
python tools/x2c_diag.py watch pfcParam.dutyFF pfcParam.piCurrent.output pfcParam.powerCommand
python tools/x2c_diag.py scope pfcParam.iL pfcParam.dutyRatio --out trace.csv
```

### Make a new set the default

Add a list next to `CHANNELS_WATCH` and point a subcommand at it:

```python
CHANNELS_LOOP = [
    "pfcParam.currentReference",
    "pfcParam.averageCurrent",
    "pfcParam.piCurrent.output",
    "pfcParam.dutyFF",
    "pfcParam.dutyRatio",
]
```

Then in `cmd_scope`, `names = args.vars or CHANNELS_DCM` becomes `... or CHANNELS_LOOP`.

### Log a variable that is not global yet

X2C-Scope can only see **globals**. A local inside a `static` function is invisible. To
expose one, add a field to `PFC_T` in [`project/pfc/pfc.h`](../project/pfc/pfc.h) and
assign it where the value is computed:

```c
float myDebugSignal;        /* in PFC_T */
...
pfcData->myDebugSignal = whatever;
```

Rebuild, **reprogram**, then `symbols myDebug` to confirm it resolved.

> Appending new fields at the end of a struct is always safe. *Deleting* or renaming one
> also means deleting its copy from `SimulinkProject/pfc_bus_copy.h`, or the SiL build stops
> compiling. (The old warning here about `project/pfc/pfc_pi.s` hardcoding `PFC_PI_T` byte
> offsets no longer applies — that assembly PI was unused and was deleted 2026-08-02.)

### Log to CSV continuously instead of to screen

`watch` prints; `scope` writes CSV but is limited to one buffer. For a long slow log,
copy `cmd_watch` and write rows to a file:

```python
import csv, datetime

def cmd_log(args):
    scope = open_scope(args.port, args.baud, ELF)
    variables = {n: scope.get_variable(n) for n in (args.vars or CHANNELS_WATCH)}
    with open(args.out, "w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["timestamp", *variables])
        try:
            while True:
                row = [v.get_value() for v in variables.values()]
                writer.writerow([datetime.datetime.now().isoformat(), *row])
                fh.flush()
                time.sleep(args.interval)
        except KeyboardInterrupt:
            print(f"\nwrote {args.out}")
    return 0
```

Register it in `main()`:

```python
p = sub.add_parser("log", help="continuous CSV log")
p.add_argument("vars", nargs="*")
p.add_argument("--out", default="log.csv")
p.add_argument("--interval", type=float, default=0.1)
p.set_defaults(func=cmd_log)
```

---

## Recipes: changing behaviour at run time

Several fields exist specifically so methods can be compared **on one build**, without
reflashing. See the comments in [`project/pfc/pfc.h`](../project/pfc/pfc.h).

| Variable | Values | Effect |
|---|---|---|
| `pfcParam.sampleCorrectionEnable` | 0 / 1 / 2 | off / ratio / valley-estimation reconstruction |
| `pfcParam.dutyFFEnable` | 0 / 1 | duty feed-forward off / on |
| `pfcParam.loadFF.enable` | 0 / 1 | load-power feed-forward off / on |
| `pfcParam.loadFF.gain` | float | feed-forward gain, bench-tuned |

Compare all three reconstruction methods back to back:

```bash
for m in 0 1 2; do
  python tools/x2c_diag.py set pfcParam.sampleCorrectionEnable $m
  python tools/x2c_diag.py scope --out method_$m.csv --factor 2
done
```

Writes are single 32-bit stores on this core, so they are atomic against the ISR. But they
take effect on the **next** ISR — do not write a value and read a dependent one in the same
breath without a short sleep.

> Writing `pfcParam.faultStatus = 0` clears the latched over-current fault and
> **re-enables the PWM outputs** on the next ISR. Useful on a bare board; do not do it with
> a power stage connected.

---

## Writing a custom script

For anything beyond the subcommands, use the library directly:

```python
import time
from mchplnet.interfaces.factory import InterfaceType
from pyx2cscope.x2cscope import X2CScope

ELF = r"C:\Users\Andy\Documents\PFC\pfc_1phases_dspic33ak\project\pfc.X\dist\default\production\pfc.X.production.elf"

x2c = X2CScope(elf_file=ELF, interface=InterfaceType.SERIAL, port="COM8", baud_rate=115200)

vdc  = x2c.get_variable("pfcParam.pfcVoltage.vdc")
mode = x2c.get_variable("pfcParam.sampleCorrectionEnable")

print(vdc.get_value())
mode.set_value(2)

# Buffered capture
x2c.clear_all_scope_channel()
for name in ("pfcParam.iL", "pfcParam.averageCurrent"):
    x2c.add_scope_channel(x2c.get_variable(name))
x2c.set_sample_time(1)              # 1 = every sample, N = every Nth
x2c.request_scope_data()
while not x2c.is_scope_data_ready():
    time.sleep(0.05)
data = x2c.get_scope_channel_data() # {name: [values]}
```

Useful API surface:

| Call | Purpose |
|---|---|
| `get_variable(name)` | Returns a `Variable`, or `None` if the name is not in the ELF |
| `.get_value()` / `.set_value(v)` | Read / write, typed automatically from DWARF |
| `list_variables()` | Every symbol name (~9400, varies per build) |
| `add_scope_channel(var)` / `clear_all_scope_channel()` | Buffered-capture channel set |
| `set_sample_time(n)` | Keep every Nth sample; trades resolution for span |
| `set_scope_trigger(TriggerConfig(var, trigger_level, trigger_mode, trigger_delay, trigger_edge))` | Trigger (edge: 0 rising, 1 falling) |
| `request_scope_data()` / `is_scope_data_ready()` / `get_scope_channel_data()` | Arm, poll, fetch |

There is also a standalone GUI (`x2cscope-gui`) shipped with the package, with the same
scope and watch views as the IDE plugin.

Offline ELF inspection, no board required:

```python
from pyx2cscope.parser.generic_parser import GenericParser
p = GenericParser(ELF)
info = p.get_var_info("pfcParam.iL")
print(info.type, info.byte_size, hex(info.address))
```

---

## Gotchas

**The ELF must match what is flashed.** Rebuild ⇒ reprogram. Otherwise addresses shift and
you read garbage that looks entirely plausible.

**`UserWarning: Loaded ELF appears incompatible ... (__GENERIC_MICROCHIP_DSPIC__)`** is
benign. `pyx2cscope` compares the ELF's device signature against the ID the target reports;
dsPIC33A reports a generic ID, so the check cannot match. Ignore it.

**The COM port number changes** whenever the USB cable moves to a different socket. The
script matches on VID:PID `04D8:00DD` instead. The Snap programmer is `04D8:9018` — a
different device, deliberately excluded.

**The port is exclusive.** If MPLAB X's X2C-Scope holds it, `probe` reports *access denied*.
Set the plugin to Disconnected, or close the IDE.

**Values freeze in `PFC_FAULT` state.** `pfcParam.iL` is only refreshed on the run path, so
in a fault it holds a stale value while the raw ADC keeps moving. When the converter is not
running, watch `pfcParam.pfcCurrent.iL` (live in every state) rather than `pfcParam.iL`.
Check `pfcParam.faultStatus` — it is a bitmask: 1 input UV, 2 input OV, 4 output OV,
8 output UV, 16 input OC. **Input OC is latching** and only clears on reset.

**Only globals are visible.** Everything of interest is inside the single global `pfcParam`
(`PFC_T`). Locals need a struct field to be observable.

---

## Why not the MPLAB X plugin

The plugin (v1.7.0) fails at *"No symbols loaded"* on this project even with
`Load symbols when programming or building for production` enabled, the alternate-hex
loading disabled, a fresh program cycle, `pfc` set as main project, and an IDE restart.

The build is provably fine: `pyx2cscope` reads roughly **9400 symbols, including all 88
`pfcParam` members**, from the same ELF — run `symbols` yourself to confirm. The fault is host-side, in a plugin that predates the dsPIC33A core and
its ELF layout. If you retry it after a plugin update, note that this firmware's
**Scope Sampletime is 62.5 µs** — the README's 50 µs is stale, and a wrong value silently
scales every time axis by 0.8. Sanity check: bus ripple must read 100 Hz.
