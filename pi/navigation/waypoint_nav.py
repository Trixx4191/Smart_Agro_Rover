"""
navigation/waypoint_nav.py
────────────────────────────
Autonomous waypoint navigation controller.
Supports multiple mission modes:
  - Patrol:  Loop through waypoints repeatedly
  - A-to-B:  Drive from first to last waypoint, then stop
  - Zone:    Boustrophedon coverage (waypoints pre-generated)
  - Return:  Navigate back to home/base position

Enhanced waypoint format:
  { lat, lon, label, action, speed, wait_sec, notes }

Actions at each waypoint:
  "sample" — stop, sample sensors, capture photo
  "photo"  — stop, capture photo only
  "wait"   — stop, wait N seconds
  "spray"  — trigger spray relay (future)
  "pass"   — just drive through, no stop
"""

import time
import math
import logging
from typing import List, Tuple, Optional, Callable, Dict, Any

log = logging.getLogger(__name__)


class WaypointNavigator:

    def __init__(self, arduino, gps, imu, logger, vision=None, dashboard=None):
        """
        Args:
            arduino    : SerialHandler instance
            gps        : GPSReader instance
            imu        : IMUReader instance
            logger     : DataLogger instance
            vision     : CropHealthAnalyzer instance (optional)
            dashboard  : DashboardServer instance (optional, for live updates)
        """
        self.arduino   = arduino
        self.gps       = gps
        self.imu       = imu
        self.logger    = logger
        self.vision    = vision
        self.dashboard = dashboard

        self._waypoints: List[Dict[str, Any]] = []
        self._current_wp   = 0
        self._running      = False
        self._paused       = False
        self._mode         = "patrol"   # patrol | atob | zone | return
        self._on_arrival: Optional[Callable] = None

        from pi.config import (
            WAYPOINT_RADIUS, SPEED_CRUISE, SPEED_TURN,
            SPEED_SLOW, OBSTACLE_DIST_CM, SAMPLE_AT_WAYPOINT
        )
        self.waypoint_radius  = WAYPOINT_RADIUS
        self.speed_cruise     = SPEED_CRUISE
        self.speed_turn       = SPEED_TURN
        self.speed_slow       = SPEED_SLOW
        self.obstacle_dist    = OBSTACLE_DIST_CM
        self.sample_at_wp     = SAMPLE_AT_WAYPOINT

    # ── Setup ─────────────────────────────────────────────────

    def load_waypoints(self, waypoints: list):
        """
        Load waypoints as list of dicts with rich data.
        Each waypoint: { lat, lon, label, action, speed, wait_sec, notes }
        Also accepts simple (lat, lon) tuples for backward compatibility.
        """
        normalized = []
        for i, wp in enumerate(waypoints):
            if isinstance(wp, (list, tuple)) and len(wp) >= 2:
                # Backward compat: (lat, lon) tuple
                normalized.append({
                    "lat": float(wp[0]), "lon": float(wp[1]),
                    "label": f"WP{i+1}", "action": "sample",
                    "speed": self.speed_slow, "wait_sec": 0, "notes": ""
                })
            elif isinstance(wp, dict):
                normalized.append({
                    "lat": float(wp.get("lat", 0)),
                    "lon": float(wp.get("lon", 0)),
                    "label": wp.get("label", f"WP{i+1}"),
                    "action": wp.get("action", "sample"),
                    "speed": wp.get("speed", self.speed_slow),
                    "wait_sec": wp.get("wait_sec", 0),
                    "notes": wp.get("notes", ""),
                })
            else:
                log.warning(f"Skipping invalid waypoint {i}: {wp}")
        self._waypoints = normalized
        self._current_wp = 0
        log.info(f"Navigator: {len(normalized)} waypoints loaded (mode={self._mode})")

    def set_mode(self, mode: str):
        """Set mission mode: patrol, atob, zone, return."""
        valid = ("patrol", "atob", "zone", "return")
        if mode not in valid:
            log.warning(f"Invalid mode '{mode}', using 'patrol'")
            mode = "patrol"
        self._mode = mode

    def on_arrival(self, fn: Callable):
        """Register a callback fired when rover reaches a waypoint."""
        self._on_arrival = fn

    # ── Control ───────────────────────────────────────────────

    def pause(self):
        self._paused = True
        self.arduino.stop()
        self._notify_dashboard(status="paused")
        log.info("Navigator: paused")

    def resume(self):
        self._paused = False
        self._notify_dashboard(status="running")
        log.info("Navigator: resumed")

    def abort(self):
        self._running = False
        self.arduino.estop()
        self._notify_dashboard(status="aborted")
        log.warning("Navigator: aborted — eStop triggered")

    # ── Main patrol loop ──────────────────────────────────────

    def run_patrol(self, loops: int = 0):
        """
        Drive through all waypoints based on the active mission mode.
        loops=0 → infinite patrol (only for patrol mode)
        loops=N → N complete laps then stop
        """
        if not self._waypoints:
            log.error("Navigator: no waypoints loaded")
            return

        self._running = True
        self._notify_dashboard(status="running", mode=self._mode)
        lap = 0

        while self._running:
            if self._mode in ("atob", "zone"):
                # One-way: drive through once, then stop
                self._run_single_pass()
                break
            elif self._mode == "return":
                self._run_return_home()
                break
            else:
                # Patrol mode: loop
                if loops > 0 and lap >= loops:
                    break
                log.info(f"Navigator: starting lap {lap + 1}")
                self._run_single_pass()
                lap += 1
                self._notify_dashboard(loops_done=lap)
                log.info(f"Navigator: lap {lap} complete")

        self.arduino.stop()
        self._notify_dashboard(status="completed")
        log.info("Navigator: patrol complete")

    def _run_single_pass(self):
        """Drive through all waypoints once."""
        for idx, wp in enumerate(self._waypoints):
            if not self._running:
                break
            self._current_wp = idx
            self._notify_dashboard(current_wp=idx, total_wp=len(self._waypoints))
            label = wp.get("label", "?")
            lat_s = f"{wp['lat']:.6f}"
            lon_s = f"{wp['lon']:.6f}"
            action = wp.get("action", "sample")
            spd = wp.get("speed", "?")
            log.info(f"Navigator: heading to waypoint {idx + 1}/{len(self._waypoints)} "
                     f"'{label}' ({lat_s}, {lon_s}) "
                     f"action={action} speed={spd}")
            self._drive_to(wp)

            if not self._running:
                break

            # Arrived — perform action at waypoint
            log.info(f"Navigator: arrived at waypoint {idx + 1}")
            self._at_waypoint(wp)

    def _run_return_home(self):
        """Navigate back to home position."""
        from pi.config import HOME_POSITION
        home = HOME_POSITION
        home_wp = {
            "lat": home["lat"], "lon": home["lon"],
            "label": home.get("label", "Home"),
            "action": "sample", "speed": self.speed_cruise,
            "wait_sec": 0, "notes": "Return to base"
        }
        self._current_wp = 0
        self._notify_dashboard(current_wp=0, total_wp=1)
        log.info(f"Navigator: returning home to ({home['lat']:.6f}, {home['lon']:.6f})")
        self._drive_to(home_wp)
        self._at_waypoint(home_wp)
        self._notify_dashboard(status="completed")

    # ── Navigation logic ──────────────────────────────────────

    def _drive_to(self, wp: Dict[str, Any]):
        """
        Drive toward a GPS waypoint using proportional heading controller.
        Uses per-waypoint speed if available.
        """
        from pi.navigation.gps_reader import bearing_to, haversine_distance

        target_lat = wp["lat"]
        target_lon = wp["lon"]
        wp_speed = wp.get("speed", self.speed_cruise)
        steer_threshold = 15    # degrees — when to correct heading
        slow_radius     = 3.0   # metres — slow down near waypoint

        while self._running:
            # Wait through pause
            if self._paused:
                self.arduino.stop()
                time.sleep(0.5)
                continue

            lat, lon = self.gps.position()
            if lat is None:
                log.warning("Navigator: no GPS fix — waiting")
                self.arduino.stop()
                time.sleep(1)
                continue

            dist    = haversine_distance(lat, lon, target_lat, target_lon)
            bearing = bearing_to(lat, lon, target_lat, target_lon)
            heading = self.imu.heading()

            # Notify dashboard with distance info
            self._notify_dashboard(
                distance_to_next=round(dist, 1),
                heading=round(heading, 1),
                bearing_to_next=round(bearing, 1),
            )

            # Arrived?
            if dist <= self.waypoint_radius:
                self.arduino.stop()
                return

            # Heading error (-180 to +180)
            error = (bearing - heading + 540) % 360 - 180

            # Choose speed — use per-waypoint speed or slow near waypoint
            if dist < slow_radius:
                speed = self.speed_slow
            else:
                speed = wp_speed

            # Steer
            if abs(error) > steer_threshold:
                direction = "right" if error > 0 else "left"
                self.arduino.move(direction, self.speed_turn)
            else:
                self.arduino.move("forward", speed)

            # Check sensor data for obstacles
            data = self.arduino.latest()
            if data.get("us_front", 999) < self.obstacle_dist:
                log.warning("Navigator: obstacle — stopping to reassess")
                self.arduino.stop()
                time.sleep(1.5)
                # Try to go around — nudge left and re-evaluate
                self.arduino.move("left", self.speed_turn)
                time.sleep(0.8)
                self.arduino.stop()

            time.sleep(0.1)   # 10Hz navigation loop

    def _at_waypoint(self, wp: Dict[str, Any]):
        """Actions taken when rover arrives at a waypoint."""
        action = wp.get("action", "sample")
        self.arduino.stop()
        time.sleep(0.5)   # settle before sampling

        stress_flag  = False
        stress_score = 0.0
        image_path   = None

        if action == "pass":
            # Just drive through — no stop
            log.info(f"Navigator: passing through '{wp.get('label', '?')}' (no action)")
            if self._on_arrival:
                self._on_arrival({
                    "waypoint": self._current_wp,
                    "lat": wp["lat"], "lon": wp["lon"],
                    "action": "pass", "sensor": {}, "stress": False,
                })
            return

        if action == "wait":
            wait_sec = wp.get("wait_sec", 5)
            log.info(f"Navigator: waiting {wait_sec}s at '{wp.get('label', '?')}'")
            time.sleep(wait_sec)

        elif action == "spray":
            log.info(f"Navigator: spraying at '{wp.get('label', '?')}'")
            # Future: trigger spray relay via Arduino command
            time.sleep(2.0)

        elif action in ("sample", "photo"):
            # Force sensor sample
            self.arduino.request_sample()
            time.sleep(0.5)
            sensor_data = self.arduino.latest()

            # Vision — capture and analyze if available
            if self.vision:
                image_path, stress_flag, stress_score = self.vision.capture_and_analyze()

            # Log everything
            self.logger.log(
                sensor_data,
                gps_coords=(wp["lat"], wp["lon"]),
                image_path=image_path,
                stress_flag=stress_flag,
                stress_score=stress_score,
            )

            if action == "sample":
                log.info(f"Navigator: sampled at '{wp.get('label', '?')}' "
                         f"stress={stress_flag} score={stress_score:.2f}")
            else:
                log.info(f"Navigator: photo at '{wp.get('label', '?')}' "
                         f"stress={stress_flag} score={stress_score:.2f}")

        # Fire arrival callback (dashboard update etc.)
        if self._on_arrival:
            sensor_data = self.arduino.latest() if action != "pass" else {}
            self._on_arrival({
                "waypoint": self._current_wp,
                "lat": wp["lat"], "lon": wp["lon"],
                "action": action,
                "sensor": sensor_data,
                "stress": stress_flag,
                "stress_score": stress_score,
            })

        if stress_flag:
            log.warning(f"Navigator: STRESS DETECTED at WP {self._current_wp + 1} "
                        f"'{wp.get('label', '?')}' (score={stress_score:.2f})")

        time.sleep(1.0)   # dwell at waypoint before moving on

    # ── Dashboard notification ─────────────────────────────────

    def _notify_dashboard(self, **kwargs):
        """Push mission state update to dashboard server."""
        if self.dashboard:
            try:
                self.dashboard.update_mission_state(**kwargs)
            except Exception:
                pass

    # ── Properties ─────────────────────────────────────────────

    @property
    def current_waypoint(self) -> int:
        return self._current_wp

    @property
    def total_waypoints(self) -> int:
        return len(self._waypoints)

    @property
    def mode(self) -> str:
        return self._mode

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def is_paused(self) -> bool:
        return self._paused
