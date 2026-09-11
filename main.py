#!/usr/bin/env python3
"""
Drone MAVLink emulator for Pi Zero 2W.

Transmits simulated drone telemetry (HEARTBEAT, SYS_STATUS, ATTITUDE, GPS_RAW_INT)
over a serial radio link and responds to GCS PING messages.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time
from math import pi

from pymavlink import mavutil


_running = True


def _handle_stop(signum, frame):
    global _running
    _running = False


signal.signal(signal.SIGTERM, _handle_stop)
signal.signal(signal.SIGINT, _handle_stop)


def wait_for_path(path: str, timeout_s: float) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline and _running:
        if os.path.exists(path):
            return True
        time.sleep(0.1)
    return os.path.exists(path)


def open_link(dev: str, baud: int, sysid: int, compid: int):
    return mavutil.mavlink_connection(
        dev,
        baud=baud,
        source_system=sysid,
        source_component=compid,
        autoreconnect=True,
        rtscts=False,
        dsrdtr=False,
    )


def send_heartbeat(mav):
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_QUADROTOR,
        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def send_sys_status(mav, batt_pct=95, vbatt_mv=11500, ibatt_ma=100):
    mav.mav.sys_status_send(
        0, 0, 0,
        500,
        int(vbatt_mv) & 0xFFFF,
        int(ibatt_ma),
        int(batt_pct),
        0, 0,
        0, 0, 0, 0,
    )


def send_attitude(mav, roll=0.0, pitch=0.0, yaw_rad=0.0):
    t_us = int(time.time() * 1e6) & 0xFFFFFFFF
    mav.mav.attitude_send(t_us, float(roll), float(pitch), float(yaw_rad), 0.0, 0.0, 0.0)


def send_gps_raw(mav, lat=40.0, lon=-74.0, alt_m=10.0, fix_type=3, eph=100, epv=100, sats=8):
    t_us = int(time.time() * 1e6)
    lat_i = int(lat * 1e7)
    lon_i = int(lon * 1e7)
    alt_mm = int(alt_m * 1000)

    try:
        mav.mav.gps_raw_int_send(
            t_us, int(fix_type), lat_i, lon_i, alt_mm,
            int(eph), int(epv), 0, 0, int(sats),
        )
    except TypeError:
        mav.mav.gps_raw_int_send(
            t_us, int(fix_type), lat_i, lon_i, alt_mm,
            int(eph), int(epv), 0, 0, int(sats),
            0, 0, 0, 0, 0, 0,
        )


def handle_rx(mav):
    """Process any incoming messages. Responds to broadcast PINGs."""
    msg = mav.recv_match(blocking=False)
    if msg is None:
        return
    msg_type = msg.get_type()
    if msg_type == "PING" and msg.target_system == 0:
        mav.mav.ping_send(
            msg.time_usec,
            msg.seq,
            msg.get_srcSystem(),
            msg.get_srcComponent(),
        )


def main() -> int:
    ap = argparse.ArgumentParser(description="Drone MAVLink emulator")
    ap.add_argument("--dev", default="/dev/ttyAMA0", help="Serial device (default: /dev/ttyAMA0)")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--sysid", type=int, default=1)
    ap.add_argument("--compid", type=int, default=1)
    ap.add_argument("--wait-dev", type=float, default=0.0,
                    help="If >0, wait up to this many seconds for --dev to exist.")
    ap.add_argument("--hb-period", type=float, default=1.0)
    ap.add_argument("--sys-period", type=float, default=2.0)
    ap.add_argument("--att-period", type=float, default=0.2)
    ap.add_argument("--gps-period", type=float, default=1.0)
    ap.add_argument("--tick", type=float, default=0.01)
    args = ap.parse_args()

    if args.wait_dev > 0:
        if not wait_for_path(args.dev, args.wait_dev):
            print(f"[drone] device did not appear: {args.dev}", file=sys.stderr)
            return 2

    mav = None

    try:
        mav = open_link(args.dev, args.baud, args.sysid, args.compid)
        print(f"[drone] MAVLink on {args.dev}@{args.baud} (sys={args.sysid} comp={args.compid})")
        print(f"[drone] HEARTBEAT {args.hb_period}s, SYS_STATUS {args.sys_period}s, "
              f"ATTITUDE {args.att_period}s, GPS {args.gps_period}s")

        t0 = time.time()
        last_hb = last_sys = last_gps = last_att = 0.0

        while _running:
            now = time.time()
            elapsed = now - t0

            # --- RX: respond to GCS messages ---
            handle_rx(mav)

            # --- TX: simulated telemetry ---
            if (now - last_hb) >= args.hb_period:
                send_heartbeat(mav)
                print(f"[drone] TX HEARTBEAT  t={elapsed:.1f}s")
                last_hb = now

            if (now - last_sys) >= args.sys_period:
                minutes = int(elapsed // 60)
                batt_pct = max(0, 95 - minutes)
                vbatt_mv = max(9000, 11500 - minutes * 10)
                send_sys_status(mav, batt_pct=batt_pct, vbatt_mv=vbatt_mv, ibatt_ma=100)
                print(f"[drone] TX SYS_STATUS batt={batt_pct}% {vbatt_mv}mV")
                last_sys = now

            if (now - last_att) >= args.att_period:
                yaw_deg = (elapsed * 10.0) % 360.0
                send_attitude(mav, yaw_rad=yaw_deg * (pi / 180.0))
                last_att = now

            if (now - last_gps) >= args.gps_period:
                lat = 40.0 + 0.00001 * (elapsed % 10)
                lon = -74.0 + 0.00001 * ((elapsed * 1.3) % 10)
                send_gps_raw(mav, lat=lat, lon=lon, alt_m=10.0, fix_type=3, eph=100, epv=100, sats=9)
                print(f"[drone] TX GPS_RAW    lat={lat:.7f} lon={lon:.7f}")
                last_gps = now

            time.sleep(args.tick)

        return 0

    except Exception as e:
        print(f"[drone] fatal error: {e}", file=sys.stderr)
        return 1

    finally:
        try:
            if mav:
                mav.close()
        except Exception:
            pass
        print("[drone] exit")


if __name__ == "__main__":
    raise SystemExit(main())
