#!/usr/bin/env python3
"""
SiK radio AT command diagnostic tool.

Queries a SiK-firmware radio (e.g. B-Cube V5) for its configuration
using AT commands, then optionally queries the remote peer via RT commands.
Useful for diagnosing pair communication failures — mismatched NETID,
AIR_SPEED, frequency band, ECC, etc.

Usage:
    python sik_query.py --dev /dev/serial/by-path/... --baud 57600
"""

import argparse
import json
import sys
import time

import serial

# Default dev path for radio_7 from config/radios.json
DEFAULT_DEV = (
    "/dev/serial/by-path/"
    "platform-fd500000.pcie-pci-0000:01:00.0-usb-0:1.2.4.1:1.0-port0"
)
DEFAULT_BAUD = 57600

# Fields that MUST match between paired radios
CRITICAL_PARAMS = {
    "S2": "AIR_SPEED",
    "S3": "NETID",
    "S5": "ECC",
    "S8": "MIN_FREQ",
    "S9": "MAX_FREQ",
    "S10": "NUM_CHANNELS",
    "S12": "LBT_RSSI",
    "S15": "MAX_WINDOW",
}


def send_command(ser, cmd, timeout=2.0):
    """Send an AT/RT command and return the response lines."""
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode())
    time.sleep(0.1)

    deadline = time.time() + timeout
    buf = b""
    while time.time() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            # SiK radios terminate responses with \r\n before the next prompt
            if buf.endswith(b"\r\n"):
                # Give a tiny bit more time for any trailing data
                time.sleep(0.05)
                tail = ser.read(256)
                if tail:
                    buf += tail
                break
        else:
            time.sleep(0.05)

    text = buf.decode(errors="replace").strip()
    # Filter out the echo of the command itself
    lines = [l.strip() for l in text.splitlines() if l.strip()]
    if lines and lines[0].upper().startswith(cmd.upper().split()[0]):
        lines = lines[1:]
    return lines


def enter_command_mode(ser):
    """Enter SiK AT command mode with the required 1s silence guard."""
    ser.reset_input_buffer()
    time.sleep(1.1)
    ser.write(b"+++")
    time.sleep(1.1)

    # Read the expected "OK" response
    buf = b""
    deadline = time.time() + 3.0
    while time.time() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf += chunk
            if b"OK" in buf:
                return True
        else:
            time.sleep(0.05)
    return False


def exit_command_mode(ser):
    """Exit AT command mode back to data mode."""
    send_command(ser, "ATO", timeout=1.0)


def parse_ati5(lines):
    """Parse ATI5 key=value parameter dump into a dict."""
    params = {}
    for line in lines:
        if "=" in line:
            parts = line.split("=", 1)
            # Format: "S2:AIR_SPEED=64"
            key_part = parts[0].strip()
            value = parts[1].strip()
            params[key_part] = value
    return params


def query_radio(ser, prefix="AT", label="LOCAL"):
    """Run the diagnostic command set (ATI/RTI series) and return results."""
    results = {}

    cmds = {
        "firmware": f"{prefix}I",
        "board_freq": f"{prefix}I3",
        "params": f"{prefix}I5",
        "rssi": f"{prefix}I7",
    }

    for key, cmd in cmds.items():
        print(f"  -> {cmd} ... ", end="", flush=True)
        resp = send_command(ser, cmd, timeout=3.0 if prefix == "RT" else 2.0)
        if resp:
            results[key] = resp
            print("OK")
        else:
            results[key] = None
            print("NO RESPONSE")

    # Print results
    print(f"\n{'=' * 50}")
    print(f"  {label} RADIO")
    print(f"{'=' * 50}")

    if results.get("firmware"):
        print(f"  Firmware:    {' '.join(results['firmware'])}")
    else:
        print("  Firmware:    (no response)")

    if results.get("board_freq"):
        print(f"  Board freq:  {' '.join(results['board_freq'])}")
    else:
        print("  Board freq:  (no response)")

    if results.get("rssi"):
        for line in results["rssi"]:
            print(f"  RSSI:        {line}")
    else:
        print("  RSSI:        (no response)")

    parsed_params = {}
    if results.get("params"):
        print(f"\n  {'Parameter':<30} {'Value':>10}")
        print(f"  {'-' * 30} {'-' * 10}")
        parsed_params = parse_ati5(results["params"])
        for key_part, value in parsed_params.items():
            # Highlight critical params
            is_critical = any(
                key_part.startswith(s) for s in CRITICAL_PARAMS
            )
            marker = " ***" if is_critical else ""
            print(f"  {key_part:<30} {value:>10}{marker}")
    else:
        print("\n  Parameters:  (no response)")

    print()
    return results, parsed_params


def compare_params(local_params, remote_params):
    """Compare critical parameters between local and remote radios."""
    print(f"{'=' * 50}")
    print("  CRITICAL PARAMETER COMPARISON")
    print(f"{'=' * 50}")
    print(f"  {'Register':<12} {'Param':<16} {'Local':>10} {'Remote':>10}  {'Status'}")
    print(f"  {'-' * 12} {'-' * 16} {'-' * 10} {'-' * 10}  {'-' * 10}")

    mismatches = []
    for reg, name in sorted(CRITICAL_PARAMS.items()):
        # Find matching keys in parsed params
        local_val = None
        remote_val = None
        search = f"{reg}:"
        for k, v in local_params.items():
            if k.startswith(search):
                local_val = v
        for k, v in remote_params.items():
            if k.startswith(search):
                remote_val = v

        l_display = local_val if local_val is not None else "?"
        r_display = remote_val if remote_val is not None else "?"

        if local_val is not None and remote_val is not None:
            if local_val == remote_val:
                status = "MATCH"
            else:
                status = "MISMATCH!"
                mismatches.append((reg, name, local_val, remote_val))
        else:
            status = "UNKNOWN"

        print(f"  {reg:<12} {name:<16} {l_display:>10} {r_display:>10}  {status}")

    print()
    if mismatches:
        print(f"  FOUND {len(mismatches)} MISMATCH(ES) — these explain the communication failure:")
        for reg, name, lv, rv in mismatches:
            print(f"    {reg} {name}: local={lv} remote={rv}")
    else:
        print("  All critical parameters match (or could not be compared).")
    print()
    return mismatches


def main():
    parser = argparse.ArgumentParser(
        description="SiK radio AT command diagnostic query"
    )
    parser.add_argument(
        "--dev", default=DEFAULT_DEV,
        help=f"Serial device path (default: radio_7)"
    )
    parser.add_argument(
        "--baud", type=int, default=DEFAULT_BAUD,
        help=f"Baud rate (default: {DEFAULT_BAUD})"
    )
    parser.add_argument(
        "--json", action="store_true",
        help="Also dump results as JSON to stdout"
    )
    args = parser.parse_args()

    print(f"SiK Radio Diagnostic Query")
    print(f"Device: {args.dev}")
    print(f"Baud:   {args.baud}")
    print()

    try:
        ser = serial.Serial(args.dev, args.baud, timeout=0.5)
    except serial.SerialException as e:
        print(f"ERROR: Cannot open serial port: {e}", file=sys.stderr)
        sys.exit(1)

    try:
        # Enter AT command mode
        print("Entering AT command mode...", end=" ", flush=True)
        if not enter_command_mode(ser):
            print("FAILED")
            print(
                "Could not enter command mode. Check that:\n"
                "  - The radio is powered on\n"
                "  - The correct serial port is specified\n"
                "  - No other process has the port open\n"
                "  - The baud rate is correct",
                file=sys.stderr,
            )
            sys.exit(1)
        print("OK\n")

        # Query local radio
        print("Querying LOCAL radio...")
        local_results, local_params = query_radio(ser, prefix="AT", label="LOCAL")

        # Query remote radio (will fail if pair isn't linked)
        print("Querying REMOTE radio (via RT commands)...")
        print("(This will fail if the radios are not currently linked)\n")
        remote_results, remote_params = query_radio(ser, prefix="RT", label="REMOTE")

        # Compare if we got params from both
        if local_params and remote_params:
            compare_params(local_params, remote_params)
        elif not remote_params:
            print(f"{'=' * 50}")
            print("  REMOTE RADIO: NOT REACHABLE")
            print(f"{'=' * 50}")
            print("  RT commands got no response — confirms the pair is NOT linked.")
            print("  To compare configs, run this script against the remote radio's")
            print("  serial port directly and compare the ATI5 output.")
            print()

        # Exit command mode
        exit_command_mode(ser)

        # Optional JSON dump
        if args.json:
            dump = {
                "local": {
                    "firmware": local_results.get("firmware"),
                    "board_freq": local_results.get("board_freq"),
                    "params": local_params,
                    "rssi": local_results.get("rssi"),
                },
                "remote": {
                    "firmware": remote_results.get("firmware"),
                    "board_freq": remote_results.get("board_freq"),
                    "params": remote_params,
                    "rssi": remote_results.get("rssi"),
                },
            }
            print("--- JSON ---")
            print(json.dumps(dump, indent=2))

    except KeyboardInterrupt:
        print("\nInterrupted.")
        exit_command_mode(ser)
    finally:
        ser.close()


if __name__ == "__main__":
    main()
