#!/usr/bin/env python3
"""
test_gps.py
───────────
GPS connection test — shows readings even with NO FIX (cold start indoors).

Use this indoors first to prove the UART + module are talking, then take
the rover outside for a real sky fix.

    python test_gps.py
    python test_gps.py --seconds 30
    python test_gps.py --port /dev/serial0 --baud 9600
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from typing import Optional, Tuple


def parse_nmea_coord(raw: str, direction: str) -> Optional[float]:
    if not raw or not direction:
        return None
    try:
        dot = raw.index(".")
        degrees = float(raw[: dot - 2])
        minutes = float(raw[dot - 2 :])
        decimal = degrees + minutes / 60.0
        if direction in ("S", "W"):
            decimal = -decimal
        return round(decimal, 7)
    except (ValueError, IndexError):
        return None


def parse_rmc(parts: list) -> dict:
    """Always return a status dict — even when fix is void."""
    out = {
        "type": "RMC",
        "status": parts[2] if len(parts) > 2 else "?",   # A=fix, V=void
        "lat": None,
        "lon": None,
        "sats": None,
        "quality": None,
    }
    if len(parts) >= 7 and parts[2] == "A":
        out["lat"] = parse_nmea_coord(parts[3], parts[4])
        out["lon"] = parse_nmea_coord(parts[5], parts[6])
    elif len(parts) >= 7 and parts[3] and parts[5]:
        # Some modules still fill lat/lon while status is V — show them
        lat = parse_nmea_coord(parts[3], parts[4])
        lon = parse_nmea_coord(parts[5], parts[6])
        if lat is not None and lon is not None:
            out["lat"] = lat
            out["lon"] = lon
    return out


def parse_gga(parts: list) -> dict:
    out = {
        "type": "GGA",
        "status": None,
        "lat": None,
        "lon": None,
        "sats": parts[7] if len(parts) > 7 else "0",
        "quality": parts[6] if len(parts) > 6 else "0",  # 0=no fix
    }
    if len(parts) >= 6:
        lat = parse_nmea_coord(parts[2], parts[3])
        lon = parse_nmea_coord(parts[4], parts[5])
        if lat is not None and lon is not None:
            out["lat"] = lat
            out["lon"] = lon
    q = out["quality"]
    out["status"] = "A" if q not in (None, "", "0") else "V"
    return out


def fmt_coord(v: Optional[float]) -> str:
    if v is None:
        return "0.0000000"   # explicit zero-style reading when unknown
    return f"{v:.7f}"


def main() -> int:
    parser = argparse.ArgumentParser(description="Farm Rover GPS connection test")
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--seconds", type=int, default=20)
    args = parser.parse_args()

    print("=" * 60)
    print("  FARM ROVER — GPS test (shows readings even with NO FIX)")
    print("=" * 60)
    print(f"  Port: {args.port}   Baud: {args.baud}   Listen: {args.seconds}s")
    print("=" * 60)

    if not os.path.exists(args.port):
        print(f"\n❌  Port not found: {args.port}")
        print("    ls -l /dev/serial0 /dev/ttyAMA0 /dev/ttyS0")
        print("    sudo raspi-config → Serial: login=No, hardware=Yes")
        return 1
    print(f"\n✅  Port exists: {args.port}")
    if os.path.islink(args.port):
        print(f"    → {os.path.realpath(args.port)}")

    try:
        import serial
    except ImportError:
        print("\n❌  pip install pyserial")
        return 1

    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except Exception as e:
        print(f"\n❌  Cannot open port: {e}")
        print("    Stop mission.py / gpsd / serial console if they hold the port")
        return 1
    print(f"✅  Opened {args.port} @ {args.baud}")
    print("\n  Live readings (lat/lon show 0.0000000 until a real fix):\n")
    print(f"  {'TIME':>6}  {'TYPE':4}  {'FIX':3}  {'SATS':>4}  {'LAT':>12}  {'LON':>12}")
    print("  " + "-" * 56)

    nmea_count = 0
    fix_count = 0
    last: dict = {
        "lat": None, "lon": None, "status": "V", "sats": "0", "quality": "0"
    }
    t0 = time.time()

    try:
        while time.time() - t0 < args.seconds:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("ascii", errors="ignore").strip()
            if not line.startswith("$"):
                continue

            nmea_count += 1
            parts = line.split(",")
            sentence = parts[0][1:] if parts[0].startswith("$") else parts[0]
            elapsed = int(time.time() - t0)

            parsed = None
            if sentence in ("GPRMC", "GNRMC"):
                parsed = parse_rmc(parts)
            elif sentence in ("GPGGA", "GNGGA"):
                parsed = parse_gga(parts)
            else:
                # still count other NMEA as proof of link
                if nmea_count <= 3 or nmea_count % 10 == 0:
                    print(f"  {elapsed:5}s  {sentence[:4]:4}  (other NMEA — link alive)")
                continue

            if parsed["lat"] is not None:
                last["lat"] = parsed["lat"]
            if parsed["lon"] is not None:
                last["lon"] = parsed["lon"]
            if parsed.get("status"):
                last["status"] = parsed["status"]
            if parsed.get("sats") is not None:
                last["sats"] = parsed["sats"] or "0"
            if parsed.get("quality") is not None:
                last["quality"] = parsed["quality"] or "0"

            has_fix = last["status"] == "A"
            if has_fix:
                fix_count += 1

            fix_tag = "YES" if has_fix else "NO"
            print(
                f"  {elapsed:5}s  {parsed['type']:4}  {fix_tag:3}  "
                f"{str(last['sats']):>4}  "
                f"{fmt_coord(last['lat']):>12}  {fmt_coord(last['lon']):>12}"
            )
    except KeyboardInterrupt:
        print("\n  (interrupted)")
    finally:
        ser.close()

    print("  " + "-" * 56)
    print("\nSummary")
    print(f"  NMEA sentences : {nmea_count}")
    print(f"  Valid fixes    : {fix_count}")
    print(f"  Last lat       : {fmt_coord(last['lat'])}")
    print(f"  Last lon       : {fmt_coord(last['lon'])}")
    print(f"  Fix status     : {'FIX' if last['status'] == 'A' else 'NO FIX (cold start / indoors OK)'}")
    print(f"  Satellites     : {last['sats']}")
    print(f"  GGA quality    : {last['quality']}  (0 = no fix, 1+ = fix)")

    if nmea_count == 0:
        print("\n❌  CONNECTION NOT VALID — no NMEA on the wire")
        print("    GPS TX → Pi GPIO15 (RX), GND, power, baud 9600, UART on")
        return 2

    print("\n✅  CONNECTION VALID — module is talking to the Pi")
    if fix_count > 0:
        print("✅  FIX OK — ready for missions")
        return 0

    print("⚠️  No satellite fix yet (normal indoors / cold start)")
    print("    Take the rover outside with clear sky, wait 30–90s, run again:")
    print(f"      python test_gps.py --seconds 60")
    return 0   # connection itself is valid


if __name__ == "__main__":
    sys.exit(main())
