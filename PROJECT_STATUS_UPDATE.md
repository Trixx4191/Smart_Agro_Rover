# Smart Agro Rover — Project Status (Up-to-date)

Last updated: 2026-06-08

This document summarizes everything currently present/implemented in the repository based on the files available in the workspace.

---

## 1) High-level goal

A low-cost autonomous farm rover for:
- Soil monitoring (moisture + soil temperature)
- Environment monitoring (air temperature, humidity, pressure)
- Crop health / stress detection (camera + OpenCV HSV analysis)
- Autonomous waypoint patrol (GPS-based loop)
- Local web dashboard for monitoring (Flask)
- Data logging to CSV + SQLite

Core architecture:
- **Raspberry Pi (Python)**: mission controller + navigation + sensing orchestration + vision + dashboard + logging
- **Arduino Nano (firmware)**: motor control + obstacle handling + sensor packet generation (simulated for dev/testing)

---

## 2) Mission controller

### `pi/mission.py`

Responsibilities:
- Initializes all subsystems
- Connects to Arduino (real serial or simulation)
- Starts GPS/IMU threads
- Starts vision analyzer
- Boots dashboard in a background thread
- Loads waypoints
- Runs the waypoint patrol loop
- Handles graceful shutdown on SIGINT/SIGTERM

Key functions:
- `build_waypoints()` returns a hardcoded list of `(lat, lon)` near Kumasi, Ghana for simulation
- `main(simulate=False)` wires the system together

Run:
- Simulation: `python -m pi.mission --simulate`
- Hardware: `python -m pi.mission`

Patrol logic integration:
- Uses `WaypointNavigator(arduino, gps, imu, logger, vision)`
- Calls `nav.load_waypoints(build_waypoints())`
- Calls `nav.run_patrol(loops=0)` (infinite)

Waypoint arrival callback:
- `on_arrival(info)` logs moisture and stress flag/score

---

## 3) Central configuration

### `pi/config.py`

Single source of truth for:
- Paths for data output (DB/CSV/images/logs)
- Serial settings (UART/baud/timeout)
- Sampling intervals / thresholds
- Sensor calibration constants
- GPS and IMU settings
- Motor speed presets
- Vision HSV thresholds and stress detection threshold
- Dashboard settings

Notable config values (as currently implemented):
- `SERIAL_PORT=/dev/ttyUSB0`, `SERIAL_BAUD=115200`
- Moisture calibration: `MOISTURE_DRY=780`, `MOISTURE_WET=330`
- Obstacle: `OBSTACLE_DIST_CM=20`
- GPS: `GPS_PORT=/dev/ttyAMA0`, `GPS_BAUD=9600`, `WAYPOINT_RADIUS=1.5` meters
- Motor speeds: `SPEED_CRUISE=150`, `SPEED_TURN=120`, `SPEED_SLOW=90`
- Vision thresholds:
  - Yellow HSV: `(20,80,80)` to `(35,255,255)`
  - Green HSV: `(36,40,40)` to `(90,255,255)`
  - `STRESS_THRESHOLD=0.25`
- Dashboard: `DASHBOARD_PORT=5000`, host `0.0.0.0`

---

## 4) Arduino ↔ Pi communications (serial link)

### `pi/comms/serial_handler.py`

Class: `SerialHandler`

Responsibilities:
- Provides a **thread-safe** interface to Arduino
- Maintains the latest sensor packet and history
- Supports both real serial and simulation mode

Key methods:
- `connect()`
  - If `simulate=True`: starts a background loop reading from the Arduino simulator state
  - Else: opens a real serial port and starts `_read_loop`
- `send(command: Dict)`
  - If simulation: calls simulator’s `handle_command()` and ingests response
  - Else: writes JSON command + newline
- `latest()` returns last full sensor packet
- `history()` returns up to last 100 sensor packets
- `request_sample()` sends `{ "cmd": "sample" }`

Read/parse behavior:
- The ingestion logic parses JSON lines.
- Marks `_ready=True` when it receives `{"status":"ready", ...}`.
- Stores packets only when they contain `moisture` or `alert` keys.

Simulated packet format includes (as currently used by other code):
- `moisture: [v1, v2]`
- `soil_temp: [t1, t2]`
- `air_temp`, `humidity`, `pressure`
- `us_front`, `us_rear`
- `enc: [lf, lr, rf, rr]`
- `estop` boolean
- `dir`, `speed`

Obstacle alert handling:
- If `alert` is present, logs warning with `dist_cm`.

---

## 5) Waypoint navigation

### `pi/navigation/waypoint_nav.py`

Class: `WaypointNavigator`

Responsibilities:
- Drive rover through GPS waypoints in order
- Use GPS distance/bearing and IMU heading to choose motion commands
- Stop when within waypoint radius
- At each waypoint:
  - request a sensor sample from Arduino
  - optionally run vision stress detection
  - log all data
  - fire an optional callback (for mission logs/dashboard)

Key initialization:
- Loads parameters from `pi.config`:
  - `WAYPOINT_RADIUS`, `SPEED_CRUISE`, `SPEED_TURN`, `SPEED_SLOW`, `OBSTACLE_DIST_CM`, `SAMPLE_AT_WAYPOINT`

Main method:
- `run_patrol(loops=0)`:
  - `loops=0` = infinite
  - For each waypoint: `_drive_to()` then `_at_waypoint()`

Navigation method:
- `_drive_to(target_lat, target_lon)`:
  - Loop until arrived
  - Uses `haversine_distance` + `bearing_to` from `gps_reader`
  - Uses `imu.heading()` for heading error
  - Controls speed: cruise vs slow near the waypoint
  - Uses a simple steering threshold (`steer_threshold=15` degrees)
  - Obstacle check:
    - If `arduino.latest().get('us_front', 999) < OBSTACLE_DIST_CM`:
      - stop, re-evaluate
      - nudge left then stop

Arrival actions:
- `_at_waypoint(lat, lon)`:
  - stop and settle
  - `arduino.request_sample()` then read `arduino.latest()`
  - if vision is provided:
    - `image_path, stress_flag, stress_score = vision.capture_and_analyze()`
  - log via `logger.log(...)`
  - call dashboard/arrival callback if registered
  - dwell 1 second

Control utilities:
- `pause()`, `resume()`, `abort()` (abort triggers `arduino.estop()`)

---

## 6) Sensor data logging

### `pi/sensors/data_logger.py`

Class: `DataLogger`

Responsibilities:
- Writes each reading to:
  1) a session CSV file
  2) a persistent SQLite database (`data/db/rover.db` by default)
- Provides query helpers for dashboard

Initialization:
- Ensures directories exist for `LOG_DIR` and DB parent
- Creates a session CSV file: `data/logs/session_<timestamp>.csv`
- Connects to SQLite DB with `check_same_thread=False`
- Creates table `readings` if not exists

CSV columns include:
- timestamp + datetime
- moisture_1, moisture_2
- soil_temp_1, soil_temp_2
- air_temp, humidity, pressure
- us_front, us_rear
- enc_lf, enc_lr, enc_rf, enc_rr
- latitude, longitude
- image_path
- stress_flag, stress_score

Core method:
- `log(sensor_data, gps_coords, image_path, stress_flag, stress_score)`

Query methods (used by dashboard/test):
- `get_recent(hours=1.0)`
- `get_last(n=10)`
- `get_stress_events()`
- `summary()`

Cleanup:
- `close()` closes CSV file and DB connection

---

## 7) Crop health / stress detection

### `pi/vision/crop_health.py`

Class: `CropHealthAnalyzer`

Responsibilities:
- Captures a crop image (or generates a synthetic test image)
- Analyzes HSV colors to estimate stress
- Produces:
  - `stress_flag` boolean
  - `stress_score` float (0.0 healthy → 1.0 stressed)
- Saves annotated debug output (side-by-side style overlay)

Simulation fallback:
- If `simulate=True`: generates a synthetic “heavily stressed” yellow image with small green patches
- If real camera init/capture fails: automatically falls back to simulation

Stress algorithm:
- Convert BGR image to HSV
- Create masks:
  - `yellow_mask` using configured yellow HSV ranges
  - `green_mask` using configured green HSV ranges
- Compute:
  - `score = yellow_px / (yellow_px + green_px)`
- `stress_flag = score > STRESS_THRESHOLD`

Annotated output:
- Overwrites yellow pixels in red-ish, green pixels in bright green
- Writes text label: `Stress: <percent>`
- Saves `*_annotated.jpg`

Public methods:
- `start()`: webcam init unless simulation
- `stop()`
- `capture_and_analyze()`
- `analyze_file(image_path)`

---

## 8) Dashboard

### `pi/vision/dashboard/server.py` (not opened in detail here)

Repo contains a Flask dashboard server. `pi/mission.py` starts it as:
- `dash = DashboardServer(logger)`
- `dash_thread = Thread(target=dash.run, kwargs={"debug": False})`
- Default mission logs: `http://localhost:5000`

Dashboard endpoints are referenced by integration tests:
- `GET /` (HTML)
- `GET /api/summary`
- `GET /api/readings`
- `GET /api/stress`

---

## 9) Arduino simulator + firmware protocol tests

### `arduino/rover_simulator.py` (referenced by serial handler + tests)

Integration relies on the simulator providing:
- background broadcast of sensor packets (every ~2 seconds)
- a `handle_command(raw_json_line)` function
- a shared mutable `state` dict used by tests

### `arduino/test_firmware.py`

Tests the Arduino command protocol against simulator logic.

Covers:
- Empty input, malformed JSON
- Unknown command
- `stop` command
- `move` command in all directions with speed clamping (0..255)
- unknown direction
- emergency stop:
  - `estop` locks behavior
  - movement commands blocked while eStop active
- `reset` clears eStop
- `sample` command returns full sensor packet fields
- sensor value ranges sanity checks
- encoder output structure

---

## 10) End-to-end integration test

### `test_integration.py`

A large end-to-end test that runs the whole system in simulation mode:
- Creates a temporary directory for:
  - SQLite DB
  - CSV logs
  - captured images
- Patches `pi.config` paths so runtime outputs go to temp
- Validates:
  1) Arduino simulator packets and command handling
  2) GPS reader produces changing positions
  3) IMU heading updates continuously
  4) DataLogger writes rows and supports queries
  5) Vision synthetic capture produces stress detections and annotated images
  6) WaypointNavigator infrastructure runs (notes about CPU contention in combined test)
  7) Dashboard endpoints return expected JSON/HTML
  8) Overall data pipeline integrity (DB reads > 5, stress events exist)

Also verifies stress events appear in SQLite.

---

## 11) Repo-level documentation

### `README.md`

Describes:
- Purpose and overview
- Project structure (Pi + Arduino + data + tests)
- Setup steps (venv, requirements install)
- How to run mission (simulation vs hardware)
- Dashboard URL
- Waypoints editing instruction
- Included Flowchart (Mermaid)
- Notes about BOM (FarmRover_BOM.xlsx)

---

## 12) Current working state / what’s clearly implemented

From the code inspected:
- Mission orchestration is implemented and runnable (simulation supported)
- Serial handling is implemented with both simulation and real serial paths
- GPS/IMU are threaded readers (their implementations exist but were not opened in this pass)
- Waypoint navigation drives to GPS waypoints and logs sensor + vision output
- Data logging supports CSV + SQLite, with query helpers
- Vision stress detection is implemented using HSV thresholds plus simulation image generation fallback
- Integration and firmware test suites exist and cover key behaviors

---

## 13) Notes / limitations observed in tests

- `test_integration.py` comments mention that full lap verification in the combined test environment can be unreliable due to CPU contention from multiple threads.
- The integration test therefore focuses on infrastructure validation rather than strict arrival timing in the combined environment.

---

## 14) Files explicitly read/confirmed in this workspace pass

- `README.md`
- `pi/mission.py`
- `pi/config.py`
- `pi/comms/serial_handler.py`
- `pi/navigation/waypoint_nav.py`
- `pi/sensors/data_logger.py`
- `pi/vision/crop_health.py`
- `test_integration.py`
- `arduino/test_firmware.py`
- `requirements.txt`

---

## Appendix A — Key run commands

1) Install dependencies
- `pip install -r requirements.txt`

2) Run simulation mission
- `python -m pi.mission --simulate`

3) Run Arduino firmware protocol tests
- `python arduino/test_firmware.py`

4) Run integration tests
- `python test_integration.py`

---

## Appendix B — What outputs are produced

During mission execution (simulation or hardware):
- `data/logs/mission_<timestamp>.log`
- `data/logs/session_<timestamp>.csv`
- `data/db/rover.db`
- `data/images/crop_<timestamp>.jpg`
- `data/images/crop_<timestamp>_annotated.jpg`

During tests:
- DB/CSV/images are written under a temp directory with config patching.

