"""
mission.py
───────────
Top-level mission controller. Wires all subsystems together
and runs the patrol loop.

Run directly on the Raspberry Pi:
    python -m pi.mission

Or with simulation mode (no hardware needed):
    python -m pi.mission --simulate
"""

import sys
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
    Define your patrol waypoints here as (lat, lon) tuples.
    Measure these with a phone GPS app walking the corners of your plot.
    For simulation, these are near Kumasi, Ghana.
    """
    return [
        (6.68850, -1.62440),   # SW corner
        (6.68860, -1.62440),   # NW corner
        (6.68860, -1.62430),   # NE corner
        (6.68850, -1.62430),   # SE corner
    ]


def main(simulate: bool = False):
    log.info("=" * 55)
    log.info("  AUTONOMOUS FARM ROVER — Mission Start")
    log.info(f"  Mode: {'SIMULATION' if simulate else 'HARDWARE'}")
    log.info("=" * 55)

    # ── Import subsystems ─────────────────────────────────────
    from pi.comms.serial_handler    import SerialHandler
    from pi.sensors.data_logger     import DataLogger
    from pi.navigation.gps_reader   import GPSReader
    from pi.navigation.imu_reader   import IMUReader
    from pi.navigation.waypoint_nav import WaypointNavigator
    from pi.vision.crop_health      import CropHealthAnalyzer
    from pi.dashboard.server        import DashboardServer

    # ── Initialise ────────────────────────────────────────────
    log.info("Initialising subsystems...")

    arduino = SerialHandler(simulate=simulate)
    gps     = GPSReader(simulate=simulate)
    imu     = IMUReader(simulate=simulate)
    logger  = DataLogger()
    vision  = CropHealthAnalyzer(simulate=simulate)
    dash    = DashboardServer(logger)

    # ── Connect ───────────────────────────────────────────────
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

    # ── Dashboard ─────────────────────────────────────────────
    dash_thread = threading.Thread(
        target=dash.run,
        kwargs={"debug": False},
        daemon=True
    )
    dash_thread.start()
    log.info("Dashboard running at http://localhost:5000")

    # ── Navigator ─────────────────────────────────────────────
    nav = WaypointNavigator(arduino, gps, imu, logger, vision)
    nav.load_waypoints(build_waypoints())

    # Optional: callback to log mission events
    def on_arrival(info):
        log.info(f"WP{info['waypoint']+1} arrived | "
                 f"m={info['sensor'].get('moisture',['?','?'])} | "
                 f"stress={info['stress']}")

    nav.on_arrival(on_arrival)

    # ── Graceful shutdown ─────────────────────────────────────
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

    # ── Run patrol ────────────────────────────────────────────
    log.info("Starting patrol — press Ctrl+C to stop safely")
    try:
        nav.run_patrol(loops=0)   # infinite loop
    except KeyboardInterrupt:
        shutdown()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Farm Rover Mission Controller")
    parser.add_argument("--simulate", action="store_true",
                        help="Run in simulation mode (no hardware required)")
    args = parser.parse_args()
    main(simulate=args.simulate)
