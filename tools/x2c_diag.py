"""X2C-Scope diagnostics and data capture for the single-phase PFC, without MPLAB X.

Talks to the target over the isolated USB-UART (J8) using the same LNet protocol the
MPLAB X X2C-Scope plugin uses, but resolves symbols straight from the production ELF.
Useful both as a bring-up diagnostic when the IDE plugin refuses to connect and as a
scripted capture path for comparing DCM/MCM reconstruction methods.

Install once:   pip install pyx2cscope

Subcommands:
    symbols [filter]   Parse the ELF only. No hardware needed.
    probe              Diagnose the serial link step by step.
    watch [vars...]    Poll variables and print them once per interval.
    scope [--out F]    Buffered high-rate capture into CSV.
    get <var>          Read one variable.
    set <var> <value>  Write one variable (e.g. sampleCorrectionEnable).
"""

import argparse
import logging
import sys
import time
from pathlib import Path

# The DWARF expression parser in pyelftools warns on some XC-DSC-emitted location
# expressions. They concern symbols we do not use; silence them so real errors show.
logging.getLogger().setLevel(logging.ERROR)

REPO = Path(__file__).resolve().parent.parent
ELF = REPO / "project" / "pfc.X" / "dist" / "default" / "production" / "pfc.X.production.elf"
BAUD = 115200

# The isolated USB-UART on J8 is an MCP2221 bridge. Windows renumbers it whenever it
# is replugged into a different socket, so match on VID:PID instead of a fixed COMn.
# The Snap programmer is 04D8:9018 and must not be picked by mistake.
TARGET_VID_PID = "04D8:00DD"
SNAP_VID_PID = "04D8:9018"

# X2CScope_Update() runs in the ADC ISR decimated by PFC_DIAGNOSTICS_DECIMATION:
#   64 kHz / 4 = 16 kHz -> 62.5 us between scope samples.
SAMPLE_PERIOD_S = 62.5e-6

# Signal chain for the mid-ON sample -> cycle-average reconstruction.
CHANNELS_DCM = [
    "pfcParam.iL",
    "pfcParam.averageCurrent",
    "pfcParam.sampleCorrFactor",
    "pfcParam.dcmDetected",
]

# Default watch set: enough to see whether the converter is alive and regulating.
CHANNELS_WATCH = [
    "pfcParam.state",
    "pfcParam.pfcVoltage.vdc",
    "pfcParam.rectifiedVac",
    "pfcParam.iL",
    "pfcParam.averageCurrent",
    "pfcParam.currentReference",
    "pfcParam.dutyRatio",
    "pfcParam.dcmDetected",
]


def find_port(preferred=None):
    """Resolve the board's USB-UART, or honour an explicit --port override."""
    import serial.tools.list_ports

    if preferred:
        return preferred

    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if TARGET_VID_PID in (port.hwid or "").upper():
            return port.device

    seen = ", ".join(f"{p.device} ({p.hwid})" for p in ports) or "none"
    raise SystemExit(
        f"No MCP2221 USB-UART ({TARGET_VID_PID}) found.\n"
        f"Ports seen: {seen}\n"
        "Check the micro-USB cable to J8, or pass --port COMn explicitly."
    )


def open_scope(port, baud, elf):
    """Connect to the target. Raises if the target does not answer."""
    from mchplnet.interfaces.factory import InterfaceType
    from pyx2cscope.x2cscope import X2CScope

    return X2CScope(elf_file=str(elf), interface=InterfaceType.SERIAL,
                    port=port, baud_rate=baud)


def cmd_symbols(args):
    """Parse the ELF with no hardware attached. Proves the build carries symbols."""
    from pyx2cscope.parser.generic_parser import GenericParser

    if not ELF.exists():
        print(f"ELF not found: {ELF}\nBuild the project first.")
        return 1

    parser = GenericParser(str(ELF))
    names = parser.get_var_list()
    print(f"ELF     : {ELF}")
    print(f"built   : {time.ctime(ELF.stat().st_mtime)}")
    print(f"family  : {parser.get_target_family()} / {parser.get_target_signature()}")
    print(f"symbols : {len(names)}")

    flt = args.filter or "pfcParam"
    hits = sorted(n for n in names if flt in n)
    print(f"\nmatching {flt!r}: {len(hits)}\n")
    for name in hits:
        info = parser.get_var_info(name)
        if info:
            print(f"  {name:42s} {info.type:20s} {info.byte_size}B @ 0x{info.address:08X}")
    return 0


def cmd_probe(args):
    """Walk the serial link one layer at a time and say which layer failed."""
    import serial
    import serial.tools.list_ports
    from mchplnet.services.frame_device_info import FrameDeviceInfo

    print("1. Serial ports present")
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("   none found - check the USB cable to J8")
        return 1
    for p in ports:
        hwid = (p.hwid or "").upper()
        if TARGET_VID_PID in hwid:
            mark = " <-- board USB-UART (J8)"
        elif SNAP_VID_PID in hwid:
            mark = " <-- Snap programmer, not this"
        else:
            mark = ""
        print(f"   {p.device:6s} {p.description}{mark}")

    print(f"\n2. Opening {args.port}")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.6)
    except Exception as exc:
        print(f"   FAILED: {exc}")
        print("   If this says 'access denied', MPLAB X still holds the port -")
        print("   set X2C-Scope to Disconnected, or close the IDE.")
        return 1
    print("   opened OK (nothing else is holding the port)")

    print("\n3. LNet device-info request")
    frame = bytes(FrameDeviceInfo().serialize())
    ser.reset_input_buffer()
    ser.write(frame)
    ser.flush()
    time.sleep(0.4)
    reply = ser.read(64)
    ser.close()

    if reply:
        print(f"   target replied with {len(reply)} bytes: {reply.hex(' ')}")
        print("   Link is good.")
        return 0

    print("   NO REPLY - the target sent nothing back.")
    print(f"""
   The PC side is fine, so the target is not running X2CScope_Communicate().
   Check, in order:

     a) Is the control board actually powered? The USB-UART bridge sits on the
        isolated USB side, so {args.port} enumerates from PC power alone even when
        the dsPIC has no supply. Port present != board powered.
     b) Is the main loop running? On the DP PIM the only MCU-driven LED is LD2
        (red) on RD11 - LD1 (green) is a power indicator and proves nothing.
        Enable MAIN_LOOP_ALIVE_BLINK in main.c: if LD2 does not flash, the ADC
        ISR is starving main() and X2CScope_Communicate() never runs.
     c) Is the target halted? If MPLAB X is in a debug session, or Snap is holding
        reset, the main loop never runs. Disconnect the debugger and power-cycle.
     d) Did the last programming operation actually take? Re-run Make and Program
        Device and watch for 'Programming complete'.""")
    return 1


def cmd_watch(args):
    """Poll a set of variables and print one line per interval."""
    names = args.vars or CHANNELS_WATCH
    scope = open_scope(args.port, args.baud, ELF)
    variables = {}
    for name in names:
        var = scope.get_variable(name)
        if var is None:
            print(f"warning: {name} not found in ELF, skipping")
            continue
        variables[name] = var

    if not variables:
        print("no readable variables")
        return 1

    labels = [n.replace("pfcParam.", "") for n in variables]
    print("  ".join(f"{lbl:>16s}" for lbl in labels))
    try:
        while True:
            row = []
            for var in variables.values():
                value = var.get_value()
                row.append(f"{value:>16.4f}" if isinstance(value, float) else f"{value!s:>16s}")
            print("  ".join(row), flush=True)
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\nstopped")
    return 0


def cmd_scope(args):
    """Buffered capture at the ISR rate, written to CSV."""
    names = args.vars or CHANNELS_DCM
    scope = open_scope(args.port, args.baud, ELF)
    scope.clear_all_scope_channel()

    for name in names:
        var = scope.get_variable(name)
        if var is None:
            print(f"warning: {name} not found in ELF, skipping")
            continue
        scope.add_scope_channel(var)

    scope.set_sample_time(args.factor)
    dt = SAMPLE_PERIOD_S * args.factor

    if args.trigger:
        from pyx2cscope.x2cscope import TriggerConfig

        trig = scope.get_variable(args.trigger)
        if trig is not None:
            scope.set_scope_trigger(TriggerConfig(trig, trigger_level=args.level,
                                                  trigger_mode=1, trigger_delay=0,
                                                  trigger_edge=args.edge))
            print(f"trigger: {args.trigger} {'rising' if args.edge == 0 else 'falling'} "
                  f"through {args.level}")

    print(f"capturing {len(names)} channels at {dt * 1e6:.1f} us/sample ...")
    scope.request_scope_data()
    deadline = time.time() + args.timeout
    while not scope.is_scope_data_ready():
        if time.time() > deadline:
            print("timed out waiting for the buffer to fill "
                  "(a trigger that never fires will do this)")
            return 1
        time.sleep(0.05)

    data = scope.get_scope_channel_data()
    depth = min(len(v) for v in data.values())
    span = depth * dt
    print(f"got {depth} samples/channel = {span * 1e3:.1f} ms")

    out = Path(args.out)
    with out.open("w", encoding="utf-8", newline="") as handle:
        handle.write("time_s," + ",".join(data.keys()) + "\n")
        for i in range(depth):
            cols = ",".join(f"{data[k][i]}" for k in data)
            handle.write(f"{i * dt:.9f},{cols}\n")
    print(f"wrote {out}")
    return 0


def cmd_get(args):
    scope = open_scope(args.port, args.baud, ELF)
    var = scope.get_variable(args.var)
    if var is None:
        print(f"{args.var} not found in ELF")
        return 1
    print(var.get_value())
    return 0


def cmd_set(args):
    scope = open_scope(args.port, args.baud, ELF)
    var = scope.get_variable(args.var)
    if var is None:
        print(f"{args.var} not found in ELF")
        return 1
    before = var.get_value()
    var.set_value(float(args.value) if "." in args.value else int(args.value))
    print(f"{args.var}: {before} -> {var.get_value()}")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None,
                    help="override the auto-detected COM port")
    ap.add_argument("--baud", type=int, default=BAUD)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("symbols", help="parse the ELF, no hardware needed")
    p.add_argument("filter", nargs="?", default=None)
    p.set_defaults(func=cmd_symbols)

    p = sub.add_parser("probe", help="diagnose the serial link")
    p.set_defaults(func=cmd_probe)

    p = sub.add_parser("watch", help="poll variables once per interval")
    p.add_argument("vars", nargs="*")
    p.add_argument("--interval", type=float, default=0.5)
    p.set_defaults(func=cmd_watch)

    p = sub.add_parser("scope", help="buffered capture to CSV")
    p.add_argument("vars", nargs="*")
    p.add_argument("--out", default="capture.csv")
    p.add_argument("--factor", type=int, default=1, help="keep every Nth sample")
    p.add_argument("--trigger", default=None, help="variable to trigger on")
    p.add_argument("--level", type=float, default=0.0)
    p.add_argument("--edge", type=int, default=0, help="0 rising, 1 falling")
    p.add_argument("--timeout", type=float, default=10.0)
    p.set_defaults(func=cmd_scope)

    p = sub.add_parser("get", help="read one variable")
    p.add_argument("var")
    p.set_defaults(func=cmd_get)

    p = sub.add_parser("set", help="write one variable")
    p.add_argument("var")
    p.add_argument("value")
    p.set_defaults(func=cmd_set)

    args = ap.parse_args(argv)
    if args.cmd != "symbols":  # the only mode that needs no hardware
        args.port = find_port(args.port)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
