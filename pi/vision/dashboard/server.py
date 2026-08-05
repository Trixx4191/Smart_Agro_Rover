"""
dashboard/server.py
────────────────────
Flask + Flask-SocketIO dashboard server.

Serves two UIs:
  /         — read-only monitoring dashboard (dashboard.html)
  /control  — manual joystick drive (control.html)
  /mission  — full Mission Control: waypoints, mission modes,
              zone coverage, live rover tracking (mission.html)

Live telemetry (rover position, mission state, path trail) is pushed
over Socket.IO from a background thread rather than polled, so
mission.html gets sub-second updates without hammering the API.
"""

import os
import json
import time
import math
import logging
import threading
from datetime import datetime

log = logging.getLogger(__name__)


class DashboardServer:

    def __init__(self, logger, arduino=None, gps=None, imu=None, nav=None, vision=None):
        self.logger  = logger
        self.arduino = arduino   # SerialHandler — enables /control and /api/mission/*
        self.gps     = gps       # GPSReader     — enables /api/gps/current, live position
        self.imu     = imu       # IMUReader     — enables live heading
        self.nav     = nav       # WaypointNavigator — enables mission control
        self.vision  = vision    # CropHealthAnalyzer — enables /video_feed

        # Mission state, updated either by WaypointNavigator's on-arrival/
        # progress callbacks (via update_mission_state) or by the mission
        # control endpoints below. This is what /api/config and the
        # 'mission_state' socket event both read from.
        self.mission_state = {
            "status": "idle",       # idle | running | paused | aborted | completed
            "mode": "patrol",
            "current_wp": 0,
            "total_wp": 0,
            "speed": 150,
            "manual_mode": False,
            "readings_mode": True,
        }
        self._mission_lock = threading.Lock()
        self._patrol_thread = None

        self._socketio = None   # set once run() creates it
        self._track_points = []
        self._track_max = 500

    # ── Called by WaypointNavigator._notify_dashboard() ──────────

    def update_mission_state(self, **kwargs):
        with self._mission_lock:
            self.mission_state.update(kwargs)
            state_copy = dict(self.mission_state)
        if self._socketio:
            self._socketio.emit("mission_state", state_copy)

    # ── Server ────────────────────────────────────────────────

    def run(self, debug: bool = False):
        from pi.config import (
            DASHBOARD_HOST, DASHBOARD_PORT, WAYPOINT_RADIUS,
            LIVE_POSITION_INTERVAL, LIVE_TRACK_MAX_POINTS,
            GOOGLE_MAPS_API_KEY, HOME_POSITION,
        )
        try:
            from flask import Flask, jsonify, send_from_directory, render_template, request
        except ImportError:
            log.error("Flask not installed — dashboard unavailable")
            return

        try:
            from flask_socketio import SocketIO
        except ImportError:
            log.error("flask-socketio not installed — run: pip install flask-socketio")
            log.error("Falling back to plain Flask — /mission live tracking won't work")
            SocketIO = None

        template_dir = os.path.join(os.path.dirname(__file__), "templates")
        app = Flask(__name__, template_folder=template_dir)

        logger  = self.logger
        arduino = self.arduino
        gps     = self.gps
        imu     = self.imu
        nav     = self.nav
        vision  = self.vision
        self._track_max = LIVE_TRACK_MAX_POINTS

        socketio = None
        if SocketIO is not None:
            socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")
            self._socketio = socketio

        VALID_DIRS = {"forward", "backward", "left", "right"}

        #  Pages 

        @app.route("/")
        def index():
            return render_template("dashboard.html")

        @app.route("/control")
        def control():
            return render_template("control.html")

        @app.route("/mission")
        def mission_page():
            return render_template("mission.html")

        #  Manual drive (yields to mission control if one is running) 

        @app.route("/api/control/move", methods=["POST"])
        def api_control_move():
            if arduino is None:
                return jsonify({"error": "no arduino handler attached"}), 503
            payload = request.get_json(silent=True) or {}
            direction = payload.get("dir")
            speed = payload.get("speed", 150)
            if direction not in VALID_DIRS:
                return jsonify({"error": "invalid dir"}), 400
            try:
                speed = max(0, min(255, int(speed)))
            except (TypeError, ValueError):
                speed = 150
            # Manual control is independent from autonomous missions. It should
            # be able to drive the rover even when no mission is active.
            self.update_mission_state(
                manual_mode=True,
                readings_mode=False,
                status="manual"
            )
            if not arduino.move(direction, speed):
                return jsonify({"error": "arduino command send failed"}), 503
            return jsonify({"ok": True, "dir": direction, "speed": speed})

        @app.route("/api/control/stop", methods=["POST"])
        def api_control_stop():
            if arduino is None:
                return jsonify({"error": "no arduino handler attached"}), 503
            self.update_mission_state(
                manual_mode=False,
                readings_mode=True,
                status="idle"
            )
            if not arduino.stop():
                return jsonify({"error": "arduino stop command send failed"}), 503
            return jsonify({"ok": True})

        @app.route("/api/control/capture", methods=["POST"])
        def api_control_capture():
            self.update_mission_state(
                manual_mode=True,
                readings_mode=False,
                status="capture"
            )

            if arduino is not None:
                try:
                    if not arduino.request_sample():
                        return jsonify({"error": "arduino capture command send failed"}), 503
                except Exception as exc:
                    log.exception("Capture request failed")
                    return jsonify({"error": str(exc)}), 500

            latest = arduino.latest() if arduino is not None else {
                "moisture": [0, 0],
                "soil_temp": [0.0, 0.0],
                "air_temp": 0.0,
                "humidity": 0.0,
                "pressure": 0.0,
                "us_front": 0,
                "us_rear": 0,
                "enc": [0, 0, 0, 0],
            }

            if vision is not None:
                try:
                    image_path, stress_flag, stress_score = vision.capture_and_analyze()
                    if logger is not None:
                        logger.log(
                            latest,
                            gps_coords=(gps.position()[0], gps.position()[1]) if gps is not None else None,
                            image_path=image_path,
                            stress_flag=stress_flag,
                            stress_score=stress_score,
                        )
                except Exception:
                    log.exception("Capture image analysis failed")
            elif logger is not None:
                try:
                    logger.log(latest, gps_coords=(gps.position()[0], gps.position()[1]) if gps is not None else None)
                except Exception:
                    log.exception("Capture logging failed")

            return jsonify({"ok": True})

        #  Live rover GPS (on-demand, used by the "Rover GPS" waypoint button) 

        @app.route("/api/gps/current", methods=["GET"])
        def api_gps_current():
            if gps is None:
                return jsonify({"error": "no gps handler attached", "has_fix": False}), 503
            lat, lon = gps.position()
            return jsonify({"lat": lat, "lon": lon, "has_fix": gps.has_fix()})

        #  Waypoints — accepts both the simple {lat,lon,label} shape from
        #  dashboard.html and the rich {lat,lon,label,action,speed,
        #  wait_sec,notes} shape from mission.html. Missing rich fields
        #  get sensible defaults so either page can read what the other wrote. 

        @app.route("/api/waypoints", methods=["GET"])
        def api_waypoints_get():
            from pi.config import WAYPOINTS_PATH
            if os.path.exists(WAYPOINTS_PATH):
                try:
                    with open(WAYPOINTS_PATH) as f:
                        return jsonify(json.load(f))
                except (json.JSONDecodeError, OSError) as e:
                    log.error(f"waypoints.json unreadable — {e}")
            # Nothing saved yet — return empty, not a pre-filled default.
            # Waypoint planning is entirely operator-driven.
            return jsonify([])

        @app.route("/api/waypoints", methods=["POST"])
        def api_waypoints_post():
            from pi.config import WAYPOINTS_PATH
            payload = request.get_json(silent=True)
            if not isinstance(payload, list):
                return jsonify({"error": "expected a list of waypoints"}), 400
            cleaned = []
            for i, wp in enumerate(payload):
                try:
                    lat = float(wp["lat"])
                    lon = float(wp["lon"])
                except (KeyError, TypeError, ValueError):
                    return jsonify({"error": f"waypoint {i} missing valid lat/lon"}), 400
                action = wp.get("action", "sample")
                if action not in ("sample", "photo", "wait", "spray", "pass"):
                    action = "sample"
                cleaned.append({
                    "lat": lat, "lon": lon,
                    "label": wp.get("label") or f"WP{i+1}",
                    "action": action,
                    "speed": int(wp.get("speed") or 90),
                    "wait_sec": int(wp.get("wait_sec") or 0),
                    "notes": wp.get("notes") or "",
                })
            os.makedirs(os.path.dirname(WAYPOINTS_PATH), exist_ok=True)
            with open(WAYPOINTS_PATH, "w") as f:
                json.dump(cleaned, f, indent=2)
            log.info(f"Saved {len(cleaned)} waypoints to {WAYPOINTS_PATH}")
            return jsonify({"ok": True, "count": len(cleaned)})

        #  Config — lets mission.html know the home position and current
        #  mission state on first load.

        @app.route("/api/config", methods=["GET"])
        def api_config():
            with self._mission_lock:
                state_copy = dict(self.mission_state)
            return jsonify({
                "home": HOME_POSITION,
                "waypoint_radius_m": WAYPOINT_RADIUS,
                "mission_state": state_copy,
            })

        #  Mission control ─

        @app.route("/api/mission/start", methods=["POST"])
        def api_mission_start():
            if nav is None:
                return jsonify({"error": "no navigator attached"}), 503
            if nav.is_running:
                return jsonify({"error": "mission already running"}), 409

            payload = request.get_json(silent=True) or {}
            mode = payload.get("mode", "patrol")
            speed = payload.get("speed")

            from pi.config import WAYPOINTS_PATH
            waypoints = []
            if os.path.exists(WAYPOINTS_PATH):
                try:
                    with open(WAYPOINTS_PATH) as f:
                        waypoints = json.load(f)
                except (json.JSONDecodeError, OSError):
                    waypoints = []

            if not waypoints:
                return jsonify({"error": "no waypoints saved — set up your mission plan "
                                          "and hit Save before starting"}), 400

            nav.set_mode(mode)
            nav.load_waypoints(waypoints)
            if speed:
                try:
                    nav.speed_cruise = max(0, min(255, int(speed)))
                except (TypeError, ValueError):
                    pass

            arduino.reset_estop()
            self.update_mission_state(
                status="running", mode=mode, current_wp=0,
                total_wp=len(waypoints), speed=nav.speed_cruise,
            )

            def _run():
                try:
                    nav.run_patrol(loops=0 if mode == "patrol" else 1)
                except Exception:
                    log.exception("Mission thread crashed")
                    self.update_mission_state(status="aborted")

            self._patrol_thread = threading.Thread(target=_run, daemon=True)
            self._patrol_thread.start()
            log.info(f"Mission started — mode={mode}")
            return jsonify({"ok": True, "mode": mode, "waypoints": len(waypoints)})

        @app.route("/api/mission/pause", methods=["POST"])
        def api_mission_pause():
            if nav is None:
                return jsonify({"error": "no navigator attached"}), 503
            nav.pause()
            self.update_mission_state(status="paused")
            return jsonify({"ok": True})

        @app.route("/api/mission/resume", methods=["POST"])
        def api_mission_resume():
            if nav is None:
                return jsonify({"error": "no navigator attached"}), 503
            nav.resume()
            self.update_mission_state(status="running")
            return jsonify({"ok": True})

        @app.route("/api/mission/abort", methods=["POST"])
        def api_mission_abort():
            if nav is None:
                return jsonify({"error": "no navigator attached"}), 503
            payload = request.get_json(silent=True) or {}
            return_home = bool(payload.get("return_home"))

            nav.abort()
            self.update_mission_state(status="aborted")

            if return_home:
                from pi.config import HOME_POSITION

                def _return():
                    # Let the aborted patrol thread fully exit and the
                    # e-stop clear before starting a fresh one-shot drive.
                    time.sleep(1.0)
                    arduino.reset_estop()
                    nav.set_mode("return")
                    nav._running = True
                    self.update_mission_state(status="returning", mode="return")
                    try:
                        nav.run_patrol(loops=1)
                    except Exception:
                        log.exception("Return-home thread crashed")
                    self.update_mission_state(status="completed")

                self._patrol_thread = threading.Thread(target=_return, daemon=True)
                self._patrol_thread.start()
                log.info(f"Mission aborted — returning home to {HOME_POSITION}")
            else:
                log.info("Mission aborted")

            return jsonify({"ok": True, "return_home": return_home})

        @app.route("/api/mission/speed", methods=["POST"])
        def api_mission_speed():
            if nav is None:
                return jsonify({"error": "no navigator attached"}), 503
            payload = request.get_json(silent=True) or {}
            try:
                speed = max(0, min(255, int(payload.get("speed", nav.speed_cruise))))
            except (TypeError, ValueError):
                return jsonify({"error": "invalid speed"}), 400
            nav.speed_cruise = speed
            self.update_mission_state(speed=speed)
            return jsonify({"ok": True, "speed": speed})

        #  Zone coverage generator (boustrophedon) 

        @app.route("/api/zone/generate", methods=["POST"])
        def api_zone_generate():
            from pi.config import SPEED_CRUISE
            payload = request.get_json(silent=True) or {}
            polygon = payload.get("polygon")
            row_spacing = payload.get("row_spacing", 2.0)
            headland = payload.get("headland", 3.0)

            if not isinstance(polygon, list) or len(polygon) < 3:
                return jsonify({"error": "polygon needs at least 3 points"}), 400
            try:
                poly_latlon = [(float(p["lat"]), float(p["lon"])) for p in polygon]
                row_spacing = max(0.5, float(row_spacing))
                headland = max(0.0, float(headland))
            except (KeyError, TypeError, ValueError):
                return jsonify({"error": "invalid polygon or spacing values"}), 400

            waypoints = _generate_zone_waypoints(
                poly_latlon, row_spacing, headland, SPEED_CRUISE
            )
            if not waypoints:
                return jsonify({"error": "polygon too small for the given spacing/headland"}), 400

            return jsonify({"ok": True, "count": len(waypoints), "waypoints": waypoints})

        #  Read-only monitoring endpoints 

        @app.route("/api/summary")
        def api_summary():
            return jsonify(logger.summary())

        @app.route("/api/readings")
        def api_readings():
            return jsonify(logger.get_last(50))

        @app.route("/api/stress")
        def api_stress():
            return jsonify(logger.get_stress_events())

        @app.route("/images/<path:filename>")
        def serve_image(filename):
            from pi.config import IMAGE_DIR
            return send_from_directory(IMAGE_DIR, filename)

        #  Live camera feed (MJPEG) — separate from the periodic crop-health
        #  captures. Browsers render multipart/x-mixed-replace natively in
        #  a plain <img> tag, no client-side JS polling needed. 

        @app.route("/video_feed")
        def video_feed():
            from flask import Response, stream_with_context

            if vision is None:
                return jsonify({"error": "no camera handler attached"}), 503

            def generate():
                target_interval = 1.0 / 6   # ~6 fps — plenty for monitoring
                # over local Wi-Fi, light enough not to starve the Pi's
                # other work (GPS/IMU/dashboard/mission control).
                while True:
                    frame = vision.get_frame_jpeg(quality=70)
                    if frame is not None:
                        yield (b"--frame\r\n"
                               b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
                    time.sleep(target_interval)

            return Response(
                stream_with_context(generate()),
                mimetype="multipart/x-mixed-replace; boundary=frame"
            )

        @app.route("/api/video_status")
        def api_video_status():
            if vision is None:
                return jsonify({"available": False})
            return jsonify({
                "available": True,
                "live": not vision.simulate and vision._camera_type is not None,
                "camera_type": vision._camera_type or "simulated",
            })

        #  Live telemetry broadcast (Socket.IO) 

        if socketio is not None and (gps is not None or imu is not None):
            def _telemetry_loop():
                while True:
                    try:
                        lat = lon = None
                        heading = None
                        if gps is not None and gps.has_fix():
                            lat, lon = gps.position()
                        if imu is not None:
                            heading = imu.heading()

                        if lat is not None and lon is not None:
                            socketio.emit("rover_position", {
                                "lat": lat, "lon": lon, "heading": heading
                            })
                            self._track_points.append({"lat": lat, "lon": lon})
                            if len(self._track_points) > self._track_max:
                                self._track_points = self._track_points[-self._track_max:]
                    except Exception:
                        log.exception("Telemetry loop error")
                    time.sleep(LIVE_POSITION_INTERVAL)

            threading.Thread(target=_telemetry_loop, daemon=True).start()

        log.info(f"Dashboard: http://{DASHBOARD_HOST}:{DASHBOARD_PORT}")
        if socketio is not None:
            socketio.run(app, host=DASHBOARD_HOST, port=DASHBOARD_PORT,
                         debug=debug, use_reloader=False, allow_unsafe_werkzeug=True)
        else:
            app.run(host=DASHBOARD_HOST, port=DASHBOARD_PORT,
                    debug=debug, use_reloader=False)


#  Zone coverage geometry (module-level — no Flask context needed) 

def _latlon_to_local_xy(lat, lon, origin_lat, origin_lon):
    """Small-area equirectangular approximation — good enough for field-sized polygons."""
    m_per_deg_lat = 110540.0
    m_per_deg_lon = 111320.0 * math.cos(math.radians(origin_lat))
    x = (lon - origin_lon) * m_per_deg_lon
    y = (lat - origin_lat) * m_per_deg_lat
    return x, y


def _local_xy_to_latlon(x, y, origin_lat, origin_lon):
    m_per_deg_lat = 110540.0
    m_per_deg_lon = 111320.0 * math.cos(math.radians(origin_lat))
    lon = origin_lon + x / m_per_deg_lon
    lat = origin_lat + y / m_per_deg_lat
    return lat, lon


def _row_intersections(poly_xy, y):
    """X-coordinates where a horizontal line at height y crosses the polygon edges."""
    xs = []
    n = len(poly_xy)
    for i in range(n):
        x1, y1 = poly_xy[i]
        x2, y2 = poly_xy[(i + 1) % n]
        if y1 == y2:
            continue
        if min(y1, y2) <= y < max(y1, y2):
            t = (y - y1) / (y2 - y1)
            xs.append(x1 + t * (x2 - x1))
    xs.sort()
    return xs


def _generate_zone_waypoints(poly_latlon, row_spacing, headland, cruise_speed):
    """
    Boustrophedon ("as the ox plows") coverage pattern: parallel rows
    spaced row_spacing metres apart, alternating direction each row so
    the rover never has to backtrack, inset from the polygon edge by
    headland metres to leave turning room.
    """
    origin_lat = sum(p[0] for p in poly_latlon) / len(poly_latlon)
    origin_lon = sum(p[1] for p in poly_latlon) / len(poly_latlon)
    poly_xy = [_latlon_to_local_xy(lat, lon, origin_lat, origin_lon) for lat, lon in poly_latlon]

    ys = [p[1] for p in poly_xy]
    y_min, y_max = min(ys) + headland, max(ys) - headland
    if y_max <= y_min:
        return []

    waypoints = []
    row_num = 0
    y = y_min
    while y <= y_max:
        xs = _row_intersections(poly_xy, y)
        # Pair up crossings into (enter, exit) spans, inset by headland
        for i in range(0, len(xs) - 1, 2):
            x_start, x_end = xs[i] + headland, xs[i + 1] - headland
            if x_end <= x_start:
                continue
            pts = [(x_start, y), (x_end, y)]
            if row_num % 2 == 1:
                pts.reverse()
            for x, yy in pts:
                lat, lon = _local_xy_to_latlon(x, yy, origin_lat, origin_lon)
                waypoints.append({
                    "lat": lat, "lon": lon,
                    "label": f"Row {row_num + 1}",
                    "action": "sample", "speed": cruise_speed,
                    "wait_sec": 0, "notes": "Zone coverage",
                })
        row_num += 1
        y += row_spacing

    return waypoints
