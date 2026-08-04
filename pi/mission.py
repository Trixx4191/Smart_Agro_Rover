"""
mission.py

Top-level mission controller. Wires all subsystems together and brings
the rover up into standby — dashboard, manual drive (/control), and
mission control (/mission) are all live immediately. Autonomous patrol
no longer starts automatically; it starts when you press "Start
Mission" on /mission (or POST /api/mission/start yourself), so the
rover never takes off driving the instant this script launches.

Run directly on the Raspberry Pi:
    python -m pi.mission

Or with simulation mode (no hardware needed):
    python -m pi.mission --simulate
"""

import sys
import os
import time
import logging
import argparse
import threading
from datetime import datetime

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler(
            f"data/logs/mission_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        ),
    ]
)
log = logging.getLogger("mission")


def build_waypoints():
    """
    Loads waypoints saved from the dashboard/mission map (data/waypoints.json).
    Returns an empty list if nothing has been saved yet — waypoint planning
    is entirely operator-driven, nothing is pre-populated at startup.
    Returns the full rich waypoint dicts (lat, lon, label, action, speed,
    wait_sec, notes) — WaypointNavigator.load_waypoints() understands
    both this and plain (lat, lon) tuples.
    """
    import json
    from pi.config import WAYPOINTS_PATH
    if os.path.exists(WAYPOINTS_PATH):
        try:
            with open(WAYPOINTS_PATH) as f:
                saved = json.load(f)
            if saved:
                log.info(f"Loaded {len(saved)} waypoints from {WAYPOINTS_PATH}")
                return saved
        except (json.JSONDecodeError, OSError, KeyError) as e:
            log.warning(f"Could not read saved waypoints ({e})")
    log.info("No waypoints saved yet — set up your mission plan at /mission")
    return []


def main(simulate: bool = False, manual: bool = False):
    log.info("=" * 55)
    log.info("  AUTONOMOUS FARM ROVER — Mission Start")
    log.info(f"  Mode: {'SIMULATION' if simulate else 'HARDWARE'}")
    log.info("=" * 55)

    if manual:
        log.info("Note: --manual is no longer needed — manual drive at /control "
                  "is always available alongside mission control at /mission. "
                  "Continuing normally.")

    #  Import subsystems 
    from pi.comms.serial_handler    import SerialHandler
    from pi.sensors.data_logger     import DataLogger
    from pi.navigation.gps_reader   import GPSReader
    from pi.navigation.imu_reader   import IMUReader
    from pi.navigation.waypoint_nav import WaypointNavigator
    from pi.vision.crop_health      import CropHealthAnalyzer
    try:
        from pi.vision.dashboard.server import DashboardServer
    except ModuleNotFoundError:
        from pi.dashboard.server import DashboardServer

    #  Initialise 
    log.info("Initialising subsystems...")

    arduino = SerialHandler(simulate=simulate)
    gps     = GPSReader(simulate=simulate)
    imu     = IMUReader(simulate=simulate)
    logger  = DataLogger()
    vision  = CropHealthAnalyzer(simulate=simulate)

    #  Connect ─
    if not arduino.connect():
        log.error("Failed to connect to Arduino — aborting")
        sys.exit(1)

    gps.start()
    imu.start()
    vision.start()

    # Wait for Arduino ready signal
    log.info("Waiting for Arduino ready...")
    timeout = time.time() + 10
    while not arduino.is_ready() and time.time() < timeout:
        time.sleep(0.2)

    if not arduino.is_ready():
        log.warning("Arduino ready signal not received — proceeding anyway")

    #  Navigator — created up front so /api/mission/* has something to
    #  drive, but nothing runs until a mission is explicitly started. 
    nav = WaypointNavigator(arduino, gps, imu, logger, vision)
    nav.load_waypoints(build_waypoints())

    def on_arrival(info):
        log.info(f"WP{info['waypoint']+1} arrived | "
                 f"action={info.get('action','?')} | "
                 f"m={info['sensor'].get('moisture',['?','?'])} | "
                 f"stress={info['stress']}")

    nav.on_arrival(on_arrival)

    #  Dashboard — wired to every subsystem so /control, /mission, and
    #  the read-only dashboard all work from this one process. 
    dash = DashboardServer(logger, arduino=arduino, gps=gps, imu=imu, nav=nav, vision=vision)
    nav.dashboard = dash   # lets WaypointNavigator push live mission_state updates

    dash_thread = threading.Thread(
        target=dash.run,
        kwargs={"debug": False},
        daemon=True
    )
    dash_thread.start()
    log.info("Dashboard running at http://localhost:5000")
    log.info("  Monitor:          /")
    log.info("  Manual drive:     /control")
    log.info("  Mission control:  /mission")

    #  Graceful shutdown 
    def shutdown():
        log.info("Shutdown signal received")
        nav.abort()
        arduino.stop()
        arduino.disconnect()
        gps.stop()
        imu.stop()
        vision.stop()
        logger.close()
        log.info("All subsystems stopped. Mission ended.")
        sys.exit(0)

    import signal
    signal.signal(signal.SIGINT,  lambda s, f: shutdown())
    signal.signal(signal.SIGTERM, lambda s, f: shutdown())

    #  Standby — wait for mission/manual commands from the web UI ─
    log.info("Standing by. Start a mission from /mission or drive manually from /control.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Farm Rover Mission Controller")
    parser.add_argument("--simulate", action="store_true",
                        help="Run in simulation mode (no hardware required)")
    parser.add_argument("--manual", action="store_true",
                        help="Deprecated — manual drive is now always available at "
                             "/control, this flag no longer changes behavior")
    args = parser.parse_args()
    main(simulate=args.simulate, manual=args.manual)
