"""
tests/test_pi.py
─────────────────
Tests for all Raspberry Pi subsystems.
Runs entirely in simulation — no hardware required.

Run: pytest tests/test_pi.py -v
  or: python tests/test_pi.py
"""

import sys
import os
import json
import time
import tempfile
import shutil

# Add rover root to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

PASS = "✅"
FAIL = "❌"
results = []


def test(name, condition, detail=""):
    status = PASS if condition else FAIL
    results.append((status, name, detail))
    suffix = f"  ({detail})" if detail and not condition else ""
    print(f"  {status}  {name}{suffix}")


def section(title):
    print(f"\n[{title}]")


# ══════════════════════════════════════════════════════════════
#  1. CONFIG
# ══════════════════════════════════════════════════════════════
section("1. Config")

from pi.config import (
    SERIAL_BAUD, SAMPLE_INTERVAL, OBSTACLE_DIST_CM,
    WAYPOINT_RADIUS, SPEED_CRUISE, SPEED_TURN, SPEED_SLOW,
    YELLOW_LOW, YELLOW_HIGH, GREEN_LOW, GREEN_HIGH,
    STRESS_THRESHOLD, DASHBOARD_PORT
)

test("SERIAL_BAUD is 115200",        SERIAL_BAUD == 115200)
test("SAMPLE_INTERVAL is positive",  SAMPLE_INTERVAL > 0)
test("OBSTACLE_DIST_CM is positive", OBSTACLE_DIST_CM > 0)
test("WAYPOINT_RADIUS is positive",  WAYPOINT_RADIUS > 0)
test("SPEED_CRUISE in 0-255",        0 < SPEED_CRUISE <= 255)
test("SPEED_TURN in 0-255",          0 < SPEED_TURN   <= 255)
test("SPEED_SLOW in 0-255",          0 < SPEED_SLOW   <= 255)
test("STRESS_THRESHOLD in 0-1",      0 < STRESS_THRESHOLD < 1)
test("YELLOW_LOW has 3 values",      len(YELLOW_LOW) == 3)
test("YELLOW_HIGH has 3 values",     len(YELLOW_HIGH) == 3)
test("GREEN_LOW has 3 values",       len(GREEN_LOW) == 3)
test("DASHBOARD_PORT is valid",      1024 < DASHBOARD_PORT < 65535)


# ══════════════════════════════════════════════════════════════
#  2. GPS READER
# ══════════════════════════════════════════════════════════════
section("2. GPS Reader")

from pi.navigation.gps_reader import (
    GPSReader, _parse_nmea_coord, _parse_gprmc, _parse_gpgga,
    haversine_distance, bearing_to
)

# NMEA coordinate parsing
lat = _parse_nmea_coord("0640.1100", "N")
test("NMEA lat parse N hemisphere",  lat is not None and lat > 0)

lat_s = _parse_nmea_coord("0640.1100", "S")
test("NMEA lat parse S hemisphere",  lat_s is not None and lat_s < 0)

lon = _parse_nmea_coord("00137.4660", "W")
test("NMEA lon parse W hemisphere",  lon is not None and lon < 0)

lon_e = _parse_nmea_coord("00137.4660", "E")
test("NMEA lon parse E hemisphere",  lon_e is not None and lon_e > 0)

test("NMEA invalid input → None",   _parse_nmea_coord("", "N") is None)

# GPRMC parsing
gprmc_parts = "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W".split(",")
result = _parse_gprmc(gprmc_parts)
test("GPRMC active fix parsed",     result is not None)
test("GPRMC lat is positive",       result and result[0] > 0)

gprmc_void = "$GPRMC,123519,V,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W".split(",")
test("GPRMC void fix → None",      _parse_gprmc(gprmc_void) is None)

# GPGGA parsing
gpgga_parts = "$GPGGA,092750.000,5321.6802,N,00630.3372,W,1,8,1.03,61.7,M,55.3,M,,*75".split(",")
result_gga = _parse_gpgga(gpgga_parts)
test("GPGGA fix parsed",            result_gga is not None)

# Haversine distance
d = haversine_distance(51.5074, -0.1278, 48.8566, 2.3522)  # London → Paris
test("Haversine London→Paris ~340km", 330000 < d < 350000, f"{d:.0f}m")

d_zero = haversine_distance(6.6885, -1.6244, 6.6885, -1.6244)
test("Haversine same point = 0",    d_zero < 0.001)

# Bearing
b = bearing_to(51.5074, -0.1278, 48.8566, 2.3522)  # London → Paris ~SE
test("Bearing London→Paris ~SE (100-180°)", 100 < b < 180, f"{b:.1f}°")

b_north = bearing_to(0.0, 0.0, 1.0, 0.0)
test("Bearing due north = 0°",      b_north < 1.0 or b_north > 359.0)

# GPSReader simulation
gps = GPSReader(simulate=True)
gps.start()
time.sleep(1.5)
lat, lon = gps.position()
test("GPS sim produces fix",        lat is not None and lon is not None)
test("GPS sim lat is float",        isinstance(lat, float))
test("GPS sim lon is float",        isinstance(lon, float))
test("GPS sim has_fix() True",      gps.has_fix())

dist = gps.distance_to(lat + 0.001, lon)
test("GPS distance_to returns metres", dist is not None and dist > 0)

brg = gps.bearing_to(lat + 0.001, lon)
test("GPS bearing_to returns degrees", brg is not None and 0 <= brg <= 360)
gps.stop()


# ══════════════════════════════════════════════════════════════
#  3. IMU READER
# ══════════════════════════════════════════════════════════════
section("3. IMU Reader")

from pi.navigation.imu_reader import IMUReader

imu = IMUReader(simulate=True)
imu.start()
time.sleep(0.5)

h = imu.heading()
test("IMU sim returns heading",     isinstance(h, float))
test("IMU heading in 0-360",        0 <= h <= 360)
test("IMU pitch is float",          isinstance(imu.pitch(), float))
test("IMU roll is float",           isinstance(imu.roll(), float))

# Heading should be incrementing (sim rotates)
h1 = imu.heading()
time.sleep(0.2)
h2 = imu.heading()
test("IMU sim heading changes",     h1 != h2)
imu.stop()


# ══════════════════════════════════════════════════════════════
#  4. SERIAL HANDLER (simulation mode)
# ══════════════════════════════════════════════════════════════
section("4. Serial Handler (sim)")

from pi.comms.serial_handler import SerialHandler

arduino = SerialHandler(simulate=True)
connected = arduino.connect()
test("SerialHandler connects in sim", connected)
test("is_ready() True after connect", arduino.is_ready())

time.sleep(2.5)   # wait for one broadcast cycle
latest = arduino.latest()
test("latest() returns dict",         isinstance(latest, dict))
test("latest() has moisture key",     "moisture" in latest)
test("latest() has soil_temp key",    "soil_temp" in latest)
test("latest() has air_temp key",     "air_temp" in latest)
test("latest() has enc key",          "enc" in latest)

# Command sending
result = arduino.send({"cmd": "move", "dir": "forward", "speed": 150})
test("send() returns True",           result is True)

result_stop = arduino.send({"cmd": "stop"})
test("stop command sends ok",         result_stop is True)

# Callback
callback_fired = []
arduino.add_callback(lambda d: callback_fired.append(d))
time.sleep(2.5)
test("Callback fired on new data",    len(callback_fired) > 0)

history = arduino.history()
test("history() returns list",        isinstance(history, list))
test("history() has entries",         len(history) > 0)

arduino.disconnect()


# ══════════════════════════════════════════════════════════════
#  5. DATA LOGGER
# ══════════════════════════════════════════════════════════════
section("5. Data Logger")

# Use a temp dir so tests don't pollute real data
tmpdir = tempfile.mkdtemp()
db_path  = os.path.join(tmpdir, "test.db")
log_dir  = os.path.join(tmpdir, "logs")

from pi.sensors.data_logger import DataLogger

logger = DataLogger(db_path=db_path, log_dir=log_dir)
test("DataLogger initialises",       logger is not None)
test("CSV file created",             os.path.exists(logger.csv_path))

sample = {
    "moisture":  [65, 72],
    "soil_temp": [24.5, 23.9],
    "air_temp":  28.1,
    "humidity":  67.3,
    "pressure":  1012.4,
    "us_front":  120,
    "us_rear":   90,
    "enc":       [10, 12, 11, 13],
}

logger.log(sample, gps_coords=(6.6885, -1.6244), image_path="images/test.jpg",
           stress_flag=False, stress_score=0.12)
logger.log(sample, gps_coords=(6.6886, -1.6244), stress_flag=True, stress_score=0.45)

last = logger.get_last(5)
test("get_last() returns list",       isinstance(last, list))
test("get_last() has 2 entries",      len(last) == 2)
test("moisture_1 stored correctly",   last[0]["moisture_1"] == 65 or last[1]["moisture_1"] == 65)
test("stress_flag stored correctly",  any(r["stress_flag"] == 1 for r in last))
test("gps lat stored",                any(r["latitude"] == 6.6885 for r in last))

stress = logger.get_stress_events()
test("get_stress_events() returns list", isinstance(stress, list))
test("stress event found",            len(stress) == 1)
test("stress_score stored",           stress[0]["stress_score"] == 0.45)

recent = logger.get_recent(hours=1)
test("get_recent() returns list",     isinstance(recent, list))
test("get_recent() finds both logs",  len(recent) == 2)

summary = logger.summary()
test("summary() returns dict",        isinstance(summary, dict))
test("summary total_readings = 2",    summary["total_readings"] == 2)
test("summary stress_events = 1",     summary["stress_events"] == 1)
test("summary avg_moisture_1 valid",  summary["avg_moisture_1"] is not None)

logger.close()

# Check CSV has data
with open(logger.csv_path) as f:
    lines = f.readlines()
test("CSV has header + 2 data rows",  len(lines) == 3)
test("CSV moisture_1 in data",        "65" in lines[1] or "65" in lines[2])

shutil.rmtree(tmpdir)


# ══════════════════════════════════════════════════════════════
#  6. VISION
# ══════════════════════════════════════════════════════════════
section("6. Vision (sim + synthetic image)")

tmpdir2 = tempfile.mkdtemp()
from pi.vision.crop_health import CropHealthAnalyzer

vision = CropHealthAnalyzer(image_dir=tmpdir2, simulate=True)
vision.start()

path, stress, score = vision.capture_and_analyze()
test("capture_and_analyze returns path",  path is not None)
test("image file exists",                 os.path.exists(path))
test("stress_flag is bool",               isinstance(stress, bool))
test("stress_score is float",             isinstance(score, float))
test("stress_score in 0-1",               0.0 <= score <= 1.0)
test("annotated image created",           os.path.exists(path.replace(".jpg","_annotated.jpg")))

# The synthetic image has yellow patches — should detect stress
test("Synthetic image detects stress",    stress is True, f"score={score:.3f}")

# Analyze an existing file
stress2, score2 = vision.analyze_file(path)
test("analyze_file() works",              isinstance(stress2, bool))
test("analyze_file score matches",        abs(score - score2) < 0.001)

vision.stop()
shutil.rmtree(tmpdir2)


# ══════════════════════════════════════════════════════════════
#  SUMMARY
# ══════════════════════════════════════════════════════════════
passed = sum(1 for s, _, __ in results if s == PASS)
failed = sum(1 for s, _, __ in results if s == FAIL)
total  = len(results)

print(f"\n{'='*55}")
print(f"  Results: {passed}/{total} passed", end="")
if failed:
    print(f"  |  {failed} FAILED")
    print("\n  Failed tests:")
    for s, name, detail in results:
        if s == FAIL:
            d = f"  ({detail})" if detail else ""
            print(f"    {FAIL}  {name}{d}")
else:
    print("  — All tests passed! 🎉")
print(f"{'='*55}\n")

sys.exit(0 if failed == 0 else 1)
