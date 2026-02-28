"""
navigation/gps_reader.py
─────────────────────────
Reads NMEA sentences from the NEO-6M GPS module over UART.
Runs a background thread — call .position() to get latest fix.

Usage:
    from pi.navigation.gps_reader import GPSReader

    gps = GPSReader()
    gps.start()

    lat, lon = gps.position()   # returns (None, None) if no fix yet
    gps.stop()
"""

import threading
import time
import math
import logging
from typing import Optional, Tuple

log = logging.getLogger(__name__)


def _parse_nmea_coord(raw: str, direction: str) -> Optional[float]:
    """Convert NMEA ddmm.mmmm format to decimal degrees."""
    if not raw or not direction:
        return None
    try:
        dot = raw.index(".")
        degrees = float(raw[:dot - 2])
        minutes = float(raw[dot - 2:])
        decimal = degrees + minutes / 60.0
        if direction in ("S", "W"):
            decimal = -decimal
        return round(decimal, 7)
    except (ValueError, IndexError):
        return None


def _parse_gprmc(parts: list) -> Optional[Tuple[float, float]]:
    """Parse $GPRMC sentence → (lat, lon) or None."""
    if len(parts) < 7 or parts[2] != "A":   # A = active fix
        return None
    lat = _parse_nmea_coord(parts[3], parts[4])
    lon = _parse_nmea_coord(parts[5], parts[6])
    if lat is not None and lon is not None:
        return lat, lon
    return None


def _parse_gpgga(parts: list) -> Optional[Tuple[float, float]]:
    """Parse $GPGGA sentence → (lat, lon) or None."""
    if len(parts) < 6 or parts[6] == "0":   # fix quality 0 = no fix
        return None
    lat = _parse_nmea_coord(parts[2], parts[3])
    lon = _parse_nmea_coord(parts[4], parts[5])
    if lat is not None and lon is not None:
        return lat, lon
    return None


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Returns distance in metres between two GPS coordinates.
    Accurate enough for short distances (< 10km).
    """
    R = 6371000   # Earth radius metres
    φ1, φ2 = math.radians(lat1), math.radians(lat2)
    Δφ = math.radians(lat2 - lat1)
    Δλ = math.radians(lon2 - lon1)
    a  = math.sin(Δφ/2)**2 + math.cos(φ1)*math.cos(φ2)*math.sin(Δλ/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing_to(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Returns bearing in degrees (0–360) from point 1 → point 2."""
    φ1, φ2 = math.radians(lat1), math.radians(lat2)
    Δλ = math.radians(lon2 - lon1)
    x  = math.sin(Δλ) * math.cos(φ2)
    y  = math.cos(φ1) * math.sin(φ2) - math.sin(φ1) * math.cos(φ2) * math.cos(Δλ)
    bearing = math.degrees(math.atan2(x, y))
    return (bearing + 360) % 360


class GPSReader:

    def __init__(self, port: str = None, baud: int = None, simulate: bool = False):
        from pi.config import GPS_PORT, GPS_BAUD
        self.port     = port or GPS_PORT
        self.baud     = baud or GPS_BAUD
        self.simulate = simulate

        self._lat: Optional[float] = None
        self._lon: Optional[float] = None
        self._lock    = threading.Lock()
        self._running = False
        self._thread  = None
        self._fix_count = 0

    def start(self):
        self._running = True
        target = self._simulate_loop if self.simulate else self._read_loop
        self._thread = threading.Thread(target=target, daemon=True)
        self._thread.start()
        log.info(f"GPSReader: started ({'sim' if self.simulate else self.port})")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)

    def position(self) -> Tuple[Optional[float], Optional[float]]:
        """Returns (lat, lon) or (None, None) if no fix."""
        with self._lock:
            return self._lat, self._lon

    def has_fix(self) -> bool:
        with self._lock:
            return self._lat is not None and self._lon is not None

    def distance_to(self, lat: float, lon: float) -> Optional[float]:
        """Distance in metres from current position to target."""
        with self._lock:
            if self._lat is None:
                return None
            return haversine_distance(self._lat, self._lon, lat, lon)

    def bearing_to(self, lat: float, lon: float) -> Optional[float]:
        """Bearing in degrees from current position to target."""
        with self._lock:
            if self._lat is None:
                return None
            return bearing_to(self._lat, self._lon, lat, lon)

    # ── Internal ──────────────────────────────────────────────

    def _read_loop(self):
        try:
            import serial
            ser = serial.Serial(self.port, self.baud, timeout=1)
        except Exception as e:
            log.error(f"GPSReader: failed to open {self.port} — {e}")
            return

        while self._running:
            try:
                line = ser.readline().decode("ascii", errors="ignore").strip()
                self._parse_line(line)
            except Exception as e:
                log.error(f"GPSReader: read error — {e}")
                time.sleep(0.5)

        ser.close()

    def _simulate_loop(self):
        """
        Simulates GPS movement along a simple grid patrol.
        Starts at a fixed origin and drifts through waypoints.
        """
        import math
        # Simulated farm origin (adjust to your test location)
        origin_lat = 6.6885    # Kumasi, Ghana approx
        origin_lon = -1.6244

        waypoints = [
            (origin_lat,         origin_lon),
            (origin_lat + 0.00010, origin_lon),
            (origin_lat + 0.00010, origin_lon + 0.00010),
            (origin_lat,           origin_lon + 0.00010),
        ]
        wp_idx = 0
        step   = 0.000002   # ~0.2m per step

        lat, lon = origin_lat, origin_lon

        while self._running:
            target_lat, target_lon = waypoints[wp_idx]
            dist = haversine_distance(lat, lon, target_lat, target_lon)

            if dist < 1.0:
                wp_idx = (wp_idx + 1) % len(waypoints)
            else:
                brg = math.radians(bearing_to(lat, lon, target_lat, target_lon))
                lat += step * math.cos(brg)
                lon += step * math.sin(brg)

            with self._lock:
                self._lat = round(lat, 7)
                self._lon = round(lon, 7)

            time.sleep(0.5)

    def _parse_line(self, line: str):
        if not line.startswith("$"):
            return
        parts = line.split(",")
        sentence = parts[0][1:]   # strip $

        result = None
        if sentence in ("GPRMC", "GNRMC"):
            result = _parse_gprmc(parts)
        elif sentence in ("GPGGA", "GNGGA"):
            result = _parse_gpgga(parts)

        if result:
            with self._lock:
                self._lat, self._lon = result
                self._fix_count += 1
            log.debug(f"GPS fix #{self._fix_count}: {self._lat}, {self._lon}")
