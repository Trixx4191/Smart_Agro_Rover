"""
navigation/waypoint_nav.py
───────────────────────────
Autonomous waypoint navigation controller.
Takes a list of GPS waypoints, drives to each in sequence,
samples sensors at each, then loops back.

Usage:
    from pi.navigation.waypoint_nav import WaypointNavigator

    nav = WaypointNavigator(arduino, gps, imu, logger)
    nav.load_waypoints([
        (6.68850, -1.62440),
        (6.68860, -1.62440),
        (6.68860, -1.62430),
        (6.68850, -1.62430),
    ])
    nav.run_patrol()   # blocking — call in a thread
"""

import time
import math
import logging
from typing import List, Tuple, Optional, Callable

log = logging.getLogger(__name__)


class WaypointNavigator:

    def __init__(self, arduino, gps, imu, logger, vision=None):
        """
        Args:
            arduino : SerialHandler instance
            gps     : GPSReader instance
            imu     : IMUReader instance
            logger  : DataLogger instance
            vision  : CropHealthAnalyzer instance (optional)
        """
        self.arduino = arduino
        self.gps     = gps
        self.imu     = imu
        self.logger  = logger
        self.vision  = vision

        self._waypoints: List[Tuple[float, float]] = []
        self._current_wp = 0
        self._running    = False
        self._paused     = False
        self._on_arrival: Optional[Callable] = None   # hook for dashboard

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

    def load_waypoints(self, waypoints: List[Tuple[float, float]]):
        """Load GPS waypoints as list of (lat, lon) tuples."""
        self._waypoints  = list(waypoints)
        self._current_wp = 0
        log.info(f"Navigator: {len(waypoints)} waypoints loaded")

    def on_arrival(self, fn: Callable):
        """Register a callback fired when rover reaches a waypoint."""
        self._on_arrival = fn

    # ── Control ───────────────────────────────────────────────

    def pause(self):
        self._paused = True
        self.arduino.stop()
        log.info("Navigator: paused")

    def resume(self):
        self._paused = False
        log.info("Navigator: resumed")

    def abort(self):
        self._running = False
        self.arduino.estop()
        log.warning("Navigator: aborted — eStop triggered")

    # ── Main patrol loop ──────────────────────────────────────

    def run_patrol(self, loops: int = 0):
        """
        Drive through all waypoints.
        loops=0 → infinite patrol, loops=N → N complete laps then stop.
        """
        if not self._waypoints:
            log.error("Navigator: no waypoints loaded")
            return

        self._running = True
        lap = 0

        while self._running:
            if loops > 0 and lap >= loops:
                break

            log.info(f"Navigator: starting lap {lap + 1}")

            for idx, (wp_lat, wp_lon) in enumerate(self._waypoints):
                if not self._running:
                    break
                self._current_wp = idx
                log.info(f"Navigator: heading to waypoint {idx + 1}/{len(self._waypoints)} "
                         f"({wp_lat:.6f}, {wp_lon:.6f})")
                self._drive_to(wp_lat, wp_lon)

                if not self._running:
                    break

                # Arrived — sample and optionally capture image
                log.info(f"Navigator: arrived at waypoint {idx + 1}")
                self._at_waypoint(wp_lat, wp_lon)

            lap += 1
            log.info(f"Navigator: lap {lap} complete")

        self.arduino.stop()
        log.info("Navigator: patrol complete")

    # ── Navigation logic ──────────────────────────────────────

    def _drive_to(self, target_lat: float, target_lon: float):
        """
        Drive toward a GPS waypoint using a simple proportional
        heading controller. Stops within WAYPOINT_RADIUS metres.
        """
        from pi.navigation.gps_reader import bearing_to, haversine_distance

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

            # Arrived?
            if dist <= self.waypoint_radius:
                self.arduino.stop()
                return

            # Heading error (-180 to +180)
            error = (bearing - heading + 540) % 360 - 180

            # Choose speed
            speed = self.speed_slow if dist < slow_radius else self.speed_cruise

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

    def _at_waypoint(self, lat: float, lon: float):
        """Actions taken when rover arrives at a waypoint."""
        self.arduino.stop()
        time.sleep(0.5)   # settle before sampling

        stress_flag  = False
        stress_score = 0.0
        image_path   = None

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
            gps_coords=(lat, lon),
            image_path=image_path,
            stress_flag=stress_flag,
            stress_score=stress_score,
        )

        # Fire arrival callback (dashboard update etc.)
        if self._on_arrival:
            self._on_arrival({
                "waypoint": self._current_wp,
                "lat": lat,
                "lon": lon,
                "sensor": sensor_data,
                "stress": stress_flag,
            })

        if stress_flag:
            log.warning(f"Navigator: STRESS DETECTED at WP {self._current_wp + 1} "
                        f"(score={stress_score:.2f})")

        time.sleep(1.0)   # dwell at waypoint before moving on

    @property
    def current_waypoint(self) -> int:
        return self._current_wp

    @property
    def total_waypoints(self) -> int:
        return len(self._waypoints)
