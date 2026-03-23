# AgroRover — Webots Farm Simulation

## What You'll See

A Clearpath Moose 4WD rover modified to match the AgroRover hardware, surveying a
50m × 50m farm field with 5 crop rows. The rover follows a boustrophedon (back-and-forth)
pattern collecting soil data at 9 waypoints while avoiding obstacles.

```
     FIELD MAP (top view)
     ┌─────────────────────────────────┐ y=+23 (north fence)
     │  🌲    🌲    🌲    🌲    🌲   │
     │  │     │     │     │     │     │  crop rows at x=-8,-4,0,4,8
     WP2│  WP3│  WP6│  WP7│     │    │  (green strips + Cypress trees)
     │  │     │     │     │     │     │
     │  ↑     ↓     ↑     ↓     │     │  rover path between rows
     │  │     │     │     │     │     │  at x = -6, -2, +2, +6
     WP1│  WP4│  WP5│  WP8│     │    │
     │  │     │     │     │     │     │
     └──┴─────┴─────┴─────┴─────┴─────┘ y=-23 (south fence)
     x=-23   x=-8  x=-4   x=0  x=4   x=14 (east fence)

     🟡 = waypoints (yellow cylinders)
     📦 = obstacles (cardboard boxes)
     🏚 = storage shed (east side)
     💧 = water tank (west side)
```

## Sensor Suite (matches real AgroRover hardware)

| Webots Sensor    | Real Hardware     | Data                          |
|------------------|-------------------|-------------------------------|
| GPS              | NEO-6M            | x, y, z position              |
| Compass          | HMC5883L          | heading vector                |
| InertialUnit     | MPU-6050          | roll, pitch, yaw              |
| Accelerometer    | MPU-6050          | ax, ay, az m/s²               |
| Gyro             | MPU-6050          | gx, gy, gz rad/s              |
| Camera 320×240   | Pi Camera v2      | live video feed               |
| DistanceSensor×2 | HC-SR04           | obstacle range in cm          |
| Simulated        | Capacitive sensor | soil moisture % (from GPS pos)|
| Simulated        | DS18B20 ×2        | soil temperature °C           |
| Simulated        | BME280            | air temp, humidity, pressure  |

---

## Installation

### Step 1 — Install Webots R2022b

Download from: https://github.com/cyberbotics/webots/releases/tag/R2022b

- Windows: run the .exe installer
- Ubuntu/Linux: `sudo apt install ./webots_2022b_amd64.deb`
- macOS: open the .dmg and drag to Applications

> ⚠️  Use R2022b specifically — the world file uses R2022b EXTERNPROTO URLs.
>     R2023+ versions may work but are untested.

### Step 2 — Extract this project

Extract the zip so your folder looks like:

```
your_folder/
├── AgroRover.wbt               ← world file
├── AgroRover.wbproj            ← project settings
└── controllers/
    └── agro_controller/
        └── agro_controller.py  ← Python controller
```

> ⚠️  The .wbt file and controllers/ folder MUST be at the same level.
>     Do not nest them inside extra subfolders.

### Step 3 — Open in Webots

1. Launch Webots
2. File → Open World
3. Navigate to your folder and select `AgroRover.wbt`
4. Webots will download ~15 PROTO files from GitHub automatically
   (requires internet connection on first load — takes 30–60 seconds)

### Step 4 — Run the simulation

1. Press ▶ (Play) at the top
2. The rover starts at WP1 (south end of Row-A)
3. Watch it drive north along Row-A, then back south along Row-B, etc.

---

## Controls During Simulation

| Action                  | How                                          |
|-------------------------|----------------------------------------------|
| Start / Pause           | ▶ / ⏸ button                               |
| Speed up simulation     | + button (up to 64× real time)               |
| Reset                   | ↺ button                                     |
| Follow rover camera     | Already set — Viewpoint follows "AgroRover"  |
| Free camera             | Click + drag in 3D view                      |
| Zoom                    | Scroll wheel                                 |
| See sensor data         | Webots menu → Tools → Console               |
| Trigger obstacle early  | Drag a CardboardBox in front of rover        |

---

## Console Output

The console prints two things every second:

### 1. Dashboard (every 2 seconds)
```
════════════════════════════════════════════════════════
  🌱 AGROROVER  t=12.0s   MODE: SURVEY
════════════════════════════════════════════════════════
  📍 POS:      x= -6.02m  y=  4.31m  h= 0.35m
  🧭 HEADING:   2.1°  N    Dist→WP: 15.7m
  🎯 WAYPOINT: 2/9 — Row-A north end
  📡 OBSTACLE: 380cm ✅ clear
  💧 MOISTURE: ch1=68%   ch2=64%
  🌡  SOIL T:  24.3°C  |  23.8°C
  🌤  AIR:     28.7°C   64%RH   1012.0hPa
  🛣  ODOMETER:24.3m   Samples: 1
════════════════════════════════════════════════════════
```

### 2. JSON packet (every 1 second — mirrors Arduino serial output)
```json
{"t":12.0,"mode":"NAV","wp":2,"pos":{"x":-6.02,"y":4.31,"z":0.35},
 "heading_deg":2.1,"imu_rpy":[0.001,-0.002,0.035],
 "moisture":[68.1,64.3],"soil_temp":[24.3,23.8],
 "air_temp":28.7,"humidity":64.2,"pressure":1012.0,
 "us_front_cm":380.0,"us_rear_cm":450.0,"estop":false,
 "obstacle":false,"odometer_m":24.3,"samples":1}
```

---

## Modifying the Simulation

### Change rover start position
In AgroRover.wbt, find the `Moose` node and edit `translation`:
```
Moose {
  translation -6 -20 0    ← change these x y z values
```

### Add more obstacles
In AgroRover.wbt, copy an existing `CardboardBox` block and change the translation:
```
CardboardBox {
  translation 2 5 0    ← x y z position
  name "obstacle_5"
  size 0.7 0.7 0.7
  mass 2
}
```

### Change crop row positions
Find `CROP_ROW_1` etc. and change the x in `translation -8 0 0.05`
Also update the `Cypress` plant translations to match.

### Adjust survey waypoints
In `agro_controller.py`, edit the `TARGETS` list:
```python
TARGETS = [
    (-6.0, -20.0),   # WP1
    ...
]
```
The corresponding yellow cylinder markers in the .wbt file should also be updated.

### Add camera image processing
In the controller, after `cam.enable(TIME_STEP * 4)`, add:
```python
# Get raw image bytes
image = cam.getImage()
width = cam.getWidth()   # 320
height = cam.getHeight() # 240
# Each pixel: R=cam.imageGetRed(image,width,x,y) etc.
```

---

## Troubleshooting

| Problem | Fix |
|---|---|
| "Controller directory not found" | Check controllers/agro_controller/ is at same level as .wbt |
| PROTO download fails | Check internet connection; Webots needs GitHub access |
| Rover falls through ground | Reset simulation (↺); physics needs 1-2 steps to settle |
| Rover drives underground | Edit Moose translation z from 0 to 0.3 in .wbt |
| "generic controller" warning | Controller folder name must match controller name exactly |
| Rover spins and doesn't navigate | Compass may need a step to initialise; normal at start |
| Simulation very slow | Menu → View → Rendering → set to Disabled for faster sim |

---

## Project Structure

```
AgroRover.wbt           World file — defines field, crops, robot
AgroRover.wbproj        Webots project settings
controllers/
  agro_controller/
    agro_controller.py  Main controller — navigation + sensors
README.md               This file
```

## Connection to Real Hardware

When physical build is complete (Phase 10), this simulation maps directly:

| Simulation                    | Real Hardware               |
|-------------------------------|-----------------------------|
| GPS.getValues()[0,1]          | gps_reader.py lat/lon       |
| compass.getValues()           | imu_reader.py heading       |
| imu.getRollPitchYaw()         | imu_reader.py roll/pitch    |
| sim_moisture() function       | sensor_hub.py moisture read |
| sim_soil_temp() function      | sensor_hub.py DS18B20 read  |
| sim_bme280() function         | sensor_hub.py BME280 read   |
| set_speed(left, right)        | serial_handler.py move cmd  |
| JSON packet print             | serial_handler.py broadcast |
| obstacle → run_avoidance()    | mission.py avoid_obstacle() |
