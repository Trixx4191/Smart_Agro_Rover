"""
config.py
──────────
Single source of truth for all rover settings.
Change values here — nothing else needs editing.
"""

import os

# ── Paths ───────────────────────────────────────────────────────
BASE_DIR    = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR    = os.path.join(BASE_DIR, "data")
LOG_DIR     = os.path.join(DATA_DIR, "logs")
DB_DIR      = os.path.join(DATA_DIR, "db")
IMAGE_DIR   = os.path.join(DATA_DIR, "images")
DB_PATH     = os.path.join(DB_DIR,   "rover.db")
WAYPOINTS_PATH    = os.path.join(DATA_DIR, "waypoints.json")
MISSIONS_PATH     = os.path.join(DATA_DIR, "missions.json")
FIELD_ZONES_PATH  = os.path.join(DATA_DIR, "field_zones.json")

# ── Map provider settings ─────────────────────────────────────────
# The dashboard now uses Leaflet tile layers by default.
# Google Maps is no longer loaded automatically as part of the normal
# workflow, so this setting can remain empty unless you add a custom
# Google Maps integration later.
GOOGLE_MAPS_API_KEY = ""

# ── Default waypoints ───────────────────────────────────────────
# Rich waypoint format with action, speed, and label.
# Actions: "sample" (stop, sample sensors, photo),
#          "photo"  (stop, photo only),
#          "wait"   (stop, wait N seconds),
#          "spray"  (trigger spray relay),
#          "pass"   (just drive through, no stop)
DEFAULT_WAYPOINTS = [
    {"lat": 6.68850, "lon": -1.62440, "label": "Plot A-1", "action": "sample", "speed": 90,  "notes": "Main plot entrance"},
    {"lat": 6.68860, "lon": -1.62440, "label": "Plot A-2", "action": "sample", "speed": 90,  "notes": "Row 3 corner"},
    {"lat": 6.68860, "lon": -1.62430, "label": "Plot B-1", "action": "photo",  "speed": 120, "notes": "Photo waypoint"},
    {"lat": 6.68850, "lon": -1.62430, "label": "Base",     "action": "wait",   "speed": 90,  "wait_sec": 5, "notes": "Return to base"},
]

# ── Mission Modes ───────────────────────────────────────────────
MISSION_MODE_PATROL      = "patrol"       # Loop through waypoints repeatedly
MISSION_MODE_ATOB        = "atob"         # A-to-B one-way then stop
MISSION_MODE_ZONE        = "zone"         # Boustrophedon coverage inside polygon
MISSION_MODE_RETURN      = "return"       # Return to base (home waypoint)

# Home position for return-to-base
HOME_POSITION = {"lat": 6.68850, "lon": -1.62440, "label": "Home"}

# Zone coverage settings
ZONE_ROW_SPACING_M   = 2.0    # metres between boustrophedon rows
ZONE_HEADLAND_M      = 3.0    # metres clearance at field edge for turning
ZONE_OVERLAP_PCT     = 10     # percentage overlap between rows

# ── Serial (Arduino ↔ Pi) ──────────────────────────────────────
SERIAL_PORT     = "/dev/ttyUSB0"   # might change to /dev/ttyACM0 if needed
SERIAL_BAUD     = 115200
SERIAL_TIMEOUT  = 2               # seconds

# ── Sensor Sampling ────────────────────────────────────────────
SAMPLE_INTERVAL     = 2.0    # seconds between Arduino broadcasts
MOISTURE_DRY        = 780    # raw ADC — dry air (calibrate on your sensor)
MOISTURE_WET        = 330    # raw ADC — fully wet (calibrate on your sensor)
OBSTACLE_DIST_CM    = 20     # stop if ultrasonic reads below this

# ── GPS (NEO-6M) ───────────────────────────────────────────────
GPS_PORT        = "/dev/serial0"   # Pi UART — always the GPIO UART on Pi4, unlike /dev/ttyAMA0 which Bluetooth can claim
GPS_BAUD        = 9600
GPS_TIMEOUT     = 1
WAYPOINT_RADIUS = 1.5              # metres — how close = "arrived"

# ── IMU (MPU-6050) ─────────────────────────────────────────────
IMU_I2C_ADDRESS = 0x68            # AD0 low = 0x68, AD0 high = 0x69

# ── Motor speeds ───────────────────────────────────────────────
SPEED_CRUISE    = 150   # 0-255 PWM — normal patrol
SPEED_TURN      = 120   # turning speed
SPEED_SLOW      = 90    # near waypoint / cautious

# ── Vision ─────────────────────────────────────────────────────
CAMERA_RESOLUTION   = (1280, 720)
CAMERA_FRAMERATE    = 30
# HSV thresholds for yellowing detection
YELLOW_LOW  = (20,  80,  80)
YELLOW_HIGH = (35, 255, 255)
# HSV thresholds for healthy green
GREEN_LOW   = (36,  40,  40)
GREEN_HIGH  = (90, 255, 255)
STRESS_THRESHOLD = 0.25    # if yellow% > 25% of leaf area → flag stress

# ── Mission ────────────────────────────────────────────────────
PATROL_INTERVAL_MIN = 120   # minutes between full patrols
MAX_MISSION_TIME    = 240   # minutes — hard cutoff for battery safety
SAMPLE_AT_WAYPOINT  = True  # force sensor sample + photo at each waypoint

# ── Dashboard ──────────────────────────────────────────────────
DASHBOARD_HOST  = "0.0.0.0"
DASHBOARD_PORT  = 5000

# ── WebSocket / Live Tracking ──────────────────────────────────
LIVE_POSITION_INTERVAL = 1.0   # seconds between rover position broadcasts
LIVE_TRACK_MAX_POINTS  = 500   # max path trail points to keep in memory
