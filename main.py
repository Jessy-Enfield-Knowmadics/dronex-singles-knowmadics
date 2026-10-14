#!/usr/bin/env python3
"""
Drone MAVLink emulator for Pi Zero 2W.

Transmits simulated drone telemetry (HEARTBEAT, SYS_STATUS, ATTITUDE,
GPS_RAW_INT, GLOBAL_POSITION_INT) over MAVLink. By default, it listens for a TCP
client, accepts MANUAL_CONTROL input, and responds to GCS PING messages. A serial
device or another pymavlink connection string can be supplied with --dev.
"""

from __future__ import annotations

import argparse
import os
import signal
import socket
import sys
import time
from dataclasses import dataclass
from math import cos, pi, radians, sin

from pymavlink import mavutil


_running = True
EARTH_RADIUS_M = 6_378_137.0
TCP_CONNECT_TIMEOUT_S = 1.0
# The controller numbers buttons from zero: bit 0 is its deadman switch and
# bit 1 is button 1, which is reserved here for toggling hover mode.
BUTTON_1_MASK = 1 << 1


@dataclass
class DroneState:
    lat: float = 40.0
    lon: float = -74.0
    altitude_m: float = 10.0
    roll_rad: float = 0.0
    pitch_rad: float = 0.0
    yaw_rad: float = 0.0
    north_mps: float = 0.0
    east_mps: float = 0.0
    down_mps: float = 0.0


@dataclass
class ManualControl:
    # MAVLink MANUAL_CONTROL conventions: x/y/r are -1000..1000 and z is
    # 0..1000, with 500 treated as altitude hold by this emulator.
    x: int = 0
    y: int = 0
    z: int = 500
    r: int = 0
    last_update: float = 0.0
    received_count: int = 0
    hover_enabled: bool = False
    button_1_pressed: bool = False


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
    previous_socket_timeout = socket.getdefaulttimeout()
    try:
        # pymavlink's outbound TCP constructor otherwise uses the operating
        # system's potentially long blocking connect timeout.
        if dev.startswith("tcp:"):
            socket.setdefaulttimeout(TCP_CONNECT_TIMEOUT_S)

        return mavutil.mavlink_connection(
            dev,
            baud=baud,
            source_system=sysid,
            source_component=compid,
            # Reconnection is owned by main() so state and failsafe handling
            # are identical for serial, TCP client, and TCP server transports.
            autoreconnect=False,
            retries=1,
            rtscts=False,
            dsrdtr=False,
        )
    finally:
        socket.setdefaulttimeout(previous_socket_timeout)


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
    time_boot_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
    mav.mav.attitude_send(time_boot_ms, float(roll), float(
        pitch), float(yaw_rad), 0.0, 0.0, 0.0)


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


def send_global_position(mav, state: DroneState, started_at: float):
    """Send the current simulated position and velocity."""
    mav.mav.global_position_int_send(
        int((time.monotonic() - started_at) * 1000) & 0xFFFFFFFF,
        int(state.lat * 1e7),
        int(state.lon * 1e7),
        int(state.altitude_m * 1000),
        int(state.altitude_m * 1000),
        int(state.north_mps * 100),
        int(state.east_mps * 100),
        int(state.down_mps * 100),
        int((state.yaw_rad * 180.0 / pi) % 360.0 * 100),
    )


def enable_tcp_nodelay(mav, connection: str) -> bool:
    """Disable Nagle's algorithm on an exposed TCP or TCP-listener socket."""
    if not connection.startswith(("tcp:", "tcpin:")):
        return False

    tcp_socket = getattr(mav, "port", None)
    if tcp_socket is None:
        # mavtcpin exposes the listener immediately and enables TCP_NODELAY on
        # every accepted client socket inside recv().
        tcp_socket = getattr(mav, "listen", None)
    if tcp_socket is None or not hasattr(tcp_socket, "setsockopt"):
        return False

    try:
        tcp_socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        return True
    except (OSError, TypeError, ValueError):
        return False


def positive_float(value: str) -> float:
    parsed = float(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def positive_int(value: str) -> int:
    parsed = int(value)
    if parsed <= 0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def advance_deadline(deadline: float, period: float, now: float) -> float:
    """Advance a periodic deadline while skipping any missed intervals."""
    intervals = max(1, int((now - deadline) // period) + 1)
    return deadline + intervals * period


def neutralize_control_after_disconnect(control: ManualControl):
    """Stop movement and require a hover-button release after reconnect."""
    control.x = 0
    control.y = 0
    control.z = 500
    control.r = 0
    control.last_update = 0.0

    # Treat button 1 as already pressed until a received message reports its
    # release. This prevents a button held across reconnect from becoming a
    # new rising edge, while preserving the current hover mode.
    control.button_1_pressed = True


def tcp_peer_disconnected(mav, connection: str, peer_was_connected: bool) -> bool:
    """Return True when a previously connected TCP peer has reached EOF."""
    if not connection.startswith(("tcp:", "tcpin:")):
        return False

    port = getattr(mav, "port", None)
    if port is None:
        # A tcpin listener has no port until its first client is accepted.
        return peer_was_connected or connection.startswith("tcp:")

    try:
        pending = port.recv(1, socket.MSG_PEEK)
    except (BlockingIOError, InterruptedError):
        return False
    except OSError:
        return True

    return pending == b""


def wait_for_retry(delay_s: float):
    """Wait for a reconnect attempt while remaining responsive to signals."""
    deadline = time.monotonic() + delay_s
    while _running:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            break
        time.sleep(min(0.1, remaining))


def update_state(
    state: DroneState,
    control: ManualControl,
    dt: float,
    now: float,
    control_timeout: float,
    max_horizontal_speed: float,
    max_climb_rate: float,
    max_yaw_rate: float,
):
    """Integrate MANUAL_CONTROL inputs into position and attitude."""
    if control.hover_enabled or now - control.last_update > control_timeout:
        x = y = r = 0.0
        z = 500.0
    else:
        x = max(-1000, min(1000, control.x))
        y = max(-1000, min(1000, control.y))
        z = max(0, min(1000, control.z))
        r = max(-1000, min(1000, control.r))

    forward_mps = (x / 1000.0) * max_horizontal_speed
    right_mps = (y / 1000.0) * max_horizontal_speed
    climb_mps = ((z - 500.0) / 500.0) * max_climb_rate
    yaw_rate_rps = (r / 1000.0) * radians(max_yaw_rate)

    state.yaw_rad = (state.yaw_rad + yaw_rate_rps * dt) % (2.0 * pi)
    state.pitch_rad = radians(-15.0 * x / 1000.0)
    state.roll_rad = radians(15.0 * y / 1000.0)

    state.north_mps = (
        forward_mps * cos(state.yaw_rad) - right_mps * sin(state.yaw_rad)
    )
    state.east_mps = (
        forward_mps * sin(state.yaw_rad) + right_mps * cos(state.yaw_rad)
    )
    state.down_mps = -climb_mps

    state.altitude_m = max(0.0, state.altitude_m + climb_mps * dt)
    state.lat += (state.north_mps * dt / EARTH_RADIUS_M) * (180.0 / pi)
    longitude_scale = max(0.01, cos(radians(state.lat)))
    state.lon += (
        state.east_mps * dt / (EARTH_RADIUS_M * longitude_scale)
    ) * (180.0 / pi)


def handle_rx(
    mav,
    control: ManualControl,
    sysid: int,
    max_messages: int,
):
    """Process a bounded RX batch, applying only its newest control input.

    Button 1 toggles hover on its rising edge. While hover is enabled, control
    messages are still consumed so another Button 1 press can restore normal
    joystick operation.
    """
    latest_control = None
    hover_toggled = False

    for _ in range(max_messages):
        msg = mav.recv_match(blocking=False)
        if msg is None:
            break

        msg_type = msg.get_type()
        if msg_type == "BAD_DATA":
            continue

        if msg_type == "PING" and getattr(msg, "target_system", 0) == 0:
            mav.mav.ping_send(
                msg.time_usec,
                msg.seq,
                msg.get_srcSystem(),
                msg.get_srcComponent(),
            )
        elif (
            msg_type == "MANUAL_CONTROL"
            and getattr(msg, "target", None) in (0, sysid)
        ):
            button_1_pressed = bool(
                getattr(msg, "buttons", 0) & BUTTON_1_MASK
            )
            if button_1_pressed and not control.button_1_pressed:
                control.hover_enabled = not control.hover_enabled
                hover_toggled = True
            control.button_1_pressed = button_1_pressed
            latest_control = msg
            control.received_count += 1

    if latest_control is not None:
        control.x = latest_control.x
        control.y = latest_control.y
        control.z = latest_control.z
        control.r = latest_control.r
        control.last_update = time.monotonic()

    return hover_toggled


def main() -> int:
    ap = argparse.ArgumentParser(description="Drone MAVLink emulator")
    ap.add_argument(
        "--dev",
        default=None,
        help=(
            "Override the MAVLink connection string, for example /dev/ttyAMA0 "
            "or tcp:192.168.1.10:5760. By default the emulator is a TCP server."
        ),
    )
    ap.add_argument("--bind-host", default="0.0.0.0",
                    help="Address used by the default TCP server (default: 0.0.0.0)")
    ap.add_argument("--port", type=int, default=5760,
                    help="Port used by the default TCP server (default: 5760)")
    ap.add_argument("--baud", type=int, default=57600)
    ap.add_argument("--sysid", type=int, default=1)
    ap.add_argument("--compid", type=int, default=1)
    ap.add_argument("--wait-dev", type=float, default=0.0,
                    help="If >0, wait up to this many seconds for --dev to exist.")
    ap.add_argument("--hb-period", type=positive_float, default=1.0)
    ap.add_argument("--sys-period", type=positive_float, default=2.0)
    ap.add_argument("--att-period", type=positive_float, default=0.01,
                    help="ATTITUDE transmission period (default: 0.01s / 100Hz)")
    ap.add_argument("--gps-period", type=positive_float, default=0.1,
                    help="GPS_RAW_INT transmission period (default: 0.1s / 10Hz)")
    ap.add_argument("--position-period", type=positive_float, default=0.02,
                    help="GLOBAL_POSITION_INT period (default: 0.02s / 50Hz)")
    ap.add_argument("--tick", type=positive_float, default=0.002,
                    help="Simulation loop period (default: 0.002s / 500Hz)")
    ap.add_argument("--control-timeout", type=positive_float, default=0.25,
                    help="Stop movement if controls are not refreshed (default: 0.25s)")
    ap.add_argument("--reconnect-delay", type=positive_float, default=1.0,
                    help="Delay between link-open attempts (default: 1.0s)")
    ap.add_argument("--max-rx-per-tick", type=positive_int, default=256,
                    help="Maximum messages processed per simulation tick (default: 256)")
    ap.add_argument("--max-horizontal-speed", type=positive_float, default=10.0,
                    help="Speed at full x/y input in m/s (default: 10)")
    ap.add_argument("--max-climb-rate", type=positive_float, default=3.0,
                    help="Climb/descent rate at full z input in m/s (default: 3)")
    ap.add_argument("--max-yaw-rate", type=positive_float, default=45.0,
                    help="Yaw rate at full r input in degrees/s (default: 45)")
    args = ap.parse_args()

    connection = args.dev or f"tcpin:{args.bind_host}:{args.port}"

    if args.wait_dev > 0 and args.dev:
        if not wait_for_path(connection, args.wait_dev):
            print(
                f"[drone] device did not appear: {connection}", file=sys.stderr)
            return 2

    print(f"[drone] HEARTBEAT {args.hb_period}s, SYS_STATUS {args.sys_period}s, "
          f"ATTITUDE {args.att_period}s, GPS {args.gps_period}s, "
          f"POSITION {args.position_period}s")

    # These objects belong to the emulator, not to any one transport session.
    # They therefore survive every disconnect and link-open retry.
    t0 = time.monotonic()
    state = DroneState()
    control = ManualControl()

    while _running:
        mav = None

        try:
            if args.dev is None:
                print(
                    f"[drone] awaiting TCP client on "
                    f"{args.bind_host}:{args.port}...",
                    flush=True,
                )

            mav = open_link(connection, args.baud, args.sysid, args.compid)
            tcp_nodelay = enable_tcp_nodelay(mav, connection)
            print(
                f"[drone] MAVLink link opened on {connection} "
                f"(sys={args.sysid} comp={args.compid})"
            )
            if connection.startswith(("tcp:", "tcpin:")):
                print(
                    f"[drone] TCP_NODELAY "
                    f"{'enabled' if tcp_nodelay else 'unavailable'}"
                )

            session_start = time.monotonic()
            previous_tick = session_start
            next_tick = session_start
            next_hb = next_sys = next_gps = next_att = next_position = (
                session_start
            )
            next_rate_report = session_start + 1.0
            last_rate_report = session_start
            last_control_count = control.received_count
            peer_was_connected = getattr(mav, "port", None) is not None

            while _running:
                now = time.monotonic()
                elapsed = now - t0
                dt = min(now - previous_tick, 0.1)
                previous_tick = now

                # --- RX: collect controller input, then update simulation ---
                hover_toggled = handle_rx(
                    mav, control, args.sysid, args.max_rx_per_tick
                )
                peer_was_connected = (
                    peer_was_connected
                    or getattr(mav, "port", None) is not None
                )
                if tcp_peer_disconnected(
                    mav, connection, peer_was_connected
                ):
                    raise ConnectionError("TCP peer disconnected")

                if hover_toggled:
                    print(
                        f"[drone] HOVER "
                        f"{'enabled' if control.hover_enabled else 'disabled'}"
                    )
                update_state(
                    state,
                    control,
                    dt,
                    now,
                    args.control_timeout,
                    args.max_horizontal_speed,
                    args.max_climb_rate,
                    args.max_yaw_rate,
                )

                # --- TX: simulated telemetry ---
                if now >= next_hb:
                    send_heartbeat(mav)
                    print(f"[drone] TX HEARTBEAT  t={elapsed:.1f}s")
                    next_hb = advance_deadline(
                        next_hb, args.hb_period, now
                    )

                if now >= next_sys:
                    minutes = int(elapsed // 60)
                    batt_pct = max(0, 95 - minutes)
                    vbatt_mv = max(9000, 11500 - minutes * 10)
                    send_sys_status(
                        mav,
                        batt_pct=batt_pct,
                        vbatt_mv=vbatt_mv,
                        ibatt_ma=100,
                    )
                    print(
                        f"[drone] TX SYS_STATUS batt={batt_pct}% "
                        f"{vbatt_mv}mV"
                    )
                    next_sys = advance_deadline(
                        next_sys, args.sys_period, now
                    )

                if now >= next_att:
                    send_attitude(
                        mav,
                        roll=state.roll_rad,
                        pitch=state.pitch_rad,
                        yaw_rad=state.yaw_rad,
                    )
                    next_att = advance_deadline(
                        next_att, args.att_period, now
                    )

                if now >= next_gps:
                    send_gps_raw(
                        mav,
                        lat=state.lat,
                        lon=state.lon,
                        alt_m=state.altitude_m,
                        fix_type=3,
                        eph=100,
                        epv=100,
                        sats=9,
                    )
                    next_gps = advance_deadline(
                        next_gps, args.gps_period, now
                    )

                if now >= next_position:
                    send_global_position(mav, state, t0)
                    next_position = advance_deadline(
                        next_position, args.position_period, now
                    )

                # A TCP write can discover a broken peer without raising.
                if tcp_peer_disconnected(
                    mav, connection, peer_was_connected
                ):
                    raise ConnectionError("TCP peer disconnected")

                if now >= next_rate_report:
                    report_period = now - last_rate_report
                    received = control.received_count - last_control_count
                    receive_rate = received / report_period
                    control_age = (
                        max(0.0, now - control.last_update)
                        if control.last_update > 0.0
                        else float("inf")
                    )
                    control_status = (
                        "fresh"
                        if control_age <= args.control_timeout
                        else "STALE"
                    )
                    print(
                        f"[drone] CONTROL {receive_rate:.1f}Hz "
                        f"{control_status} "
                        f"mode="
                        f"{'HOVER' if control.hover_enabled else 'MANUAL'} "
                        f"age={control_age * 1000:.1f}ms | "
                        f"lat={state.lat:.7f} lon={state.lon:.7f} "
                        f"alt={state.altitude_m:.2f}m "
                        f"roll={state.roll_rad * 180.0 / pi:.1f}deg "
                        f"pitch={state.pitch_rad * 180.0 / pi:.1f}deg "
                        f"yaw={state.yaw_rad * 180.0 / pi:.1f}deg"
                    )
                    last_rate_report = now
                    last_control_count = control.received_count
                    next_rate_report = advance_deadline(
                        next_rate_report, 1.0, now
                    )

                next_tick += args.tick
                remaining = next_tick - time.monotonic()
                if remaining > 0:
                    time.sleep(remaining)
                else:
                    # Skip missed ticks rather than running a stale burst.
                    next_tick = time.monotonic()

        except Exception as e:
            neutralize_control_after_disconnect(control)
            if _running:
                print(f"[drone] link lost: {e}", file=sys.stderr)

        finally:
            try:
                if mav:
                    mav.close()
            except Exception:
                pass

        if _running:
            print(
                f"[drone] retrying {connection} in "
                f"{args.reconnect_delay:.1f}s"
            )
            wait_for_retry(args.reconnect_delay)

    print("[drone] exit")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
