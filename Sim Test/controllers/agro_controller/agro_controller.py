"""
AgroRover — Webots Farm Simulation Controller
==============================================
Modified Clearpath Moose chassis carrying:
  Hardware mirrors:
    GPS          → NEO-6M (position accuracy ±0.5m)
    Compass      → HMC5883L magnetometer
    IMU          → MPU-6050 (roll/pitch/yaw)
    Accelerometer→ MPU-6050 accel axes
    Gyro         → MPU-6050 gyro axes
    Camera       → Raspberry Pi Camera Module v2 (320x240)
    us_front     → HC-SR04 front obstacle detection
    us_rear      → HC-SR04 rear clearance
    soil moisture→ simulated from GPS position (capacitive sensors)
    soil temp    → simulated from GPS position (DS18B20)
    air/hum/pres → simulated BME280 readings

Mission: Boustrophedon (back-and-forth) soil survey
  Rover drives between 5 crop rows collecting samples
  at 9 waypoints covering the 50m × 50m field

Navigation algorithm:
  Same as Webots Moose sample (moose_path_following.c)
  GPS[0]=X(east), GPS[1]=Y(north), compass for heading

Coordinate system (Z-up):
  X = East   Y = North   Z = Up (height)
"""

from controller import Robot
import math
import json
import time

# ── Constants ─────────────────────────────────────────────────
TIME_STEP          = 16       # ms  matches WorldInfo basicTimeStep
MAX_SPEED          = 7.0      # rad/s
TURN_COEFFICIENT   = 4.0      # steering gain
DISTANCE_TOLERANCE = 1.5      # m — waypoint reached threshold
OBSTACLE_DIST_M    = 1.0      # m — trigger avoidance
SAMPLE_INTERVAL    = 3.0      # s  — sensor broadcast period

# ── Boustrophedon survey waypoints  (x=east, y=north) ──────────
# Rover drives between crop rows at x = -6, -2, 2, 6
# Field boundary: ±20m north, crop rows at x = -8,-4,0,4,8
TARGETS = [
    (-6.0, -20.0),    # WP1  Row-A south end
    (-6.0,  20.0),    # WP2  Row-A north end
    (-2.0,  20.0),    # WP3  Row-B north end
    (-2.0, -20.0),    # WP4  Row-B south end
    ( 2.0, -20.0),    # WP5  Row-C south end
    ( 2.0,  20.0),    # WP6  Row-C north end
    ( 6.0,  20.0),    # WP7  Row-D north end
    ( 6.0, -20.0),    # WP8  Row-D south end
    ( 0.0,    0.0),    # WP9  Return to centre / home
]

WP_NAMES = [
    "Row-A south",  "Row-A north",
    "Row-B north",  "Row-B south",
    "Row-C south",  "Row-C north",
    "Row-D north",  "Row-D south",
    "HOME",
]

# ── Robot initialisation ───────────────────────────────────────
robot = Robot()

# 8 drive motors (same names as Clearpath Moose PROTO)
_motor_names = [
    "left motor 1",  "left motor 2",  "left motor 3",  "left motor 4",
    "right motor 1", "right motor 2", "right motor 3", "right motor 4",
]
motors = []
for mn in _motor_names:
    m = robot.getDevice(mn)
    m.setPosition(float("inf"))   # velocity-control mode
    m.setVelocity(0.0)
    motors.append(m)

# Sensors
gps   = robot.getDevice("gps");   gps.enable(TIME_STEP)
comp  = robot.getDevice("compass"); comp.enable(TIME_STEP)
imu   = robot.getDevice("imu");   imu.enable(TIME_STEP)
accel = robot.getDevice("accelerometer"); accel.enable(TIME_STEP)
gyro  = robot.getDevice("gyro");  gyro.enable(TIME_STEP)
cam   = robot.getDevice("camera"); cam.enable(TIME_STEP * 4)  # camera at 1/4 rate
usf   = robot.getDevice("us_front"); usf.enable(TIME_STEP)
usr   = robot.getDevice("us_rear");  usr.enable(TIME_STEP)

# ── Motor helpers ──────────────────────────────────────────────
def set_speed(left: float, right: float):
    left  = max(-MAX_SPEED, min(MAX_SPEED, left))
    right = max(-MAX_SPEED, min(MAX_SPEED, right))
    for i in range(4):
        motors[i].setVelocity(left)
        motors[i + 4].setVelocity(right)

def stop():                    set_speed(0.0, 0.0)
def forward(s=MAX_SPEED):      set_speed(s, s)
def backward(s=MAX_SPEED*0.6): set_speed(-s, -s)
def spin_left(s=MAX_SPEED*0.5):  set_speed(-s,  s)
def spin_right(s=MAX_SPEED*0.5): set_speed( s, -s)

# ── Navigation math (direct port from sample C controller) ─────
def modulus_double(a: float, m: float) -> float:
    r = a - int(a / m) * m
    if r < 0.0:
        r += m
    return r

def compute_steering(gps_vals, compass_vals, target):
    """
    Returns (left_speed, right_speed, distance_to_target)
    Identical algorithm to moose_path_following.c
    """
    x, y = gps_vals[0], gps_vals[1]   # horizontal position
    tx, ty = target

    du, dv = tx - x, ty - y
    distance = math.sqrt(du*du + dv*dv)
    if distance < 1e-6:
        return 0.0, 0.0, 0.0

    # Heading from compass (same as sample)
    robot_angle  = math.atan2(compass_vals[0], compass_vals[1])
    target_angle = math.atan2(dv, du)
    beta = modulus_double(target_angle - robot_angle, 2.0 * math.pi) - math.pi

    if beta > 0:
        beta =  math.pi - beta
    else:
        beta = -beta - math.pi

    left  = MAX_SPEED - math.pi + TURN_COEFFICIENT * beta
    right = MAX_SPEED - math.pi - TURN_COEFFICIENT * beta
    return left, right, distance

# ── Sensor simulation (mirrors Arduino firmware output) ────────
def sim_moisture(x: float, y: float, t: float):
    """Capacitive soil moisture — varies with distance to crop rows"""
    # Crop rows at x = -8, -4, 0, 4, 8
    nearest = round(x / 4.0) * 4.0
    dist_to_row = abs(x - nearest)
    base = 72.0 - dist_to_row * 9.0
    noise1 = 3.0 * math.sin(t * 0.3 + y * 0.1)
    noise2 = 2.5 * math.cos(t * 0.25 + x * 0.1)
    ch1 = max(5.0, min(98.0, base + noise1))
    ch2 = max(5.0, min(98.0, base - 4.0 + noise2))
    return round(ch1, 1), round(ch2, 1)

def sim_soil_temp(x: float, y: float, t: float):
    """DS18B20 soil temperature — cooler near water tank (x=-20)"""
    dist_water = abs(x + 20.0)
    t1 = 24.5 - dist_water * 0.05 + 0.3 * math.sin(t * 0.05)
    t2 = 23.8 - dist_water * 0.04 + 0.2 * math.cos(t * 0.07)
    return round(t1, 2), round(t2, 2)

def sim_bme280(t: float):
    """BME280 air temperature, humidity, pressure"""
    air  = 28.5  + 0.8 * math.sin(t * 0.02)
    hum  = 64.0  + 3.0 * math.cos(t * 0.015)
    pres = 1012.0 - 0.5 * abs(math.sin(t * 0.01))
    return round(air, 1), round(hum, 1), round(pres, 1)

# ── State ──────────────────────────────────────────────────────
class State:
    wp_idx       = 0
    avoiding     = False
    avoid_step   = 0
    avoid_phase  = "back"    # "back" → "turn" → "forward" → done
    estop        = False
    samples      = []
    step_count   = 0
    last_print   = -99.0
    last_json    = -99.0
    dist_total   = 0.0
    last_xy      = None

S = State()

# ── Obstacle avoidance state machine ──────────────────────────
def run_avoidance():
    S.avoid_step += 1
    if   S.avoid_phase == "back"    and S.avoid_step < 25:
        backward()
    elif S.avoid_phase == "back":
        S.avoid_phase = "turn"
        S.avoid_step  = 0
    elif S.avoid_phase == "turn"   and S.avoid_step < 35:
        spin_right()
    elif S.avoid_phase == "turn":
        S.avoid_phase  = "forward"
        S.avoid_step   = 0
    elif S.avoid_phase == "forward" and S.avoid_step < 20:
        forward(MAX_SPEED * 0.7)
    else:
        # avoidance complete — resume navigation
        S.avoiding    = False
        S.avoid_step  = 0
        S.avoid_phase = "back"

# ── Console dashboard ──────────────────────────────────────────
COMPASS_LABELS = ["N","NNE","NE","ENE","E","ESE","SE","SSE",
                  "S","SSW","SW","WSW","W","WNW","NW","NNW","N"]

def print_dashboard(t, gps_v, comp_v, usf_m, m1, m2, t1, t2, air, hum, pres):
    x, y = gps_v[0], gps_v[1]
    heading_deg = (math.degrees(math.atan2(comp_v[0], comp_v[1])) % 360)
    compass_idx = round(heading_deg / 22.5) % 16
    compass_lbl = COMPASS_LABELS[compass_idx]

    wp_num   = S.wp_idx + 1
    wp_name  = WP_NAMES[S.wp_idx] if S.wp_idx < len(WP_NAMES) else "DONE"
    mode     = "AVOIDING" if S.avoiding else ("COMPLETE" if S.wp_idx >= len(TARGETS) else "SURVEY")

    if S.wp_idx < len(TARGETS):
        tx, ty   = TARGETS[S.wp_idx]
        dist_wp  = math.sqrt((tx-x)**2 + (ty-y)**2)
    else:
        dist_wp = 0.0

    print(f"\n{'═'*56}")
    print(f"  🌱 AGROROVER  t={t:.1f}s   MODE: {mode}")
    print(f"{'═'*56}")
    print(f"  📍 POS:      x={x:+6.2f}m  y={y:+6.2f}m  h={gps_v[2]:.2f}m")
    print(f"  🧭 HEADING:  {heading_deg:5.1f}°  {compass_lbl:<4}  Dist→WP: {dist_wp:.1f}m")
    print(f"  🎯 WAYPOINT: {wp_num}/{len(TARGETS)} — {wp_name}")
    print(f"  📡 OBSTACLE: {usf_m*100:.0f}cm {'⚠️  AVOID' if S.avoiding else '✅ clear'}")
    print(f"  💧 MOISTURE: ch1={m1:.0f}%   ch2={m2:.0f}%")
    print(f"  🌡  SOIL T:  {t1:.1f}°C  |  {t2:.1f}°C")
    print(f"  🌤  AIR:     {air:.1f}°C   {hum:.0f}%RH   {pres:.1f}hPa")
    print(f"  🛣  ODOMETER:{S.dist_total:.1f}m   Samples: {len(S.samples)}")
    if S.estop:
        print("  🛑 EMERGENCY STOP ACTIVE")
    print(f"{'═'*56}")

# ── Sample recording (mirrors data_pipeline.py CSV) ───────────
def record_sample(t, x, y, m1, m2, t1, t2, air, hum, pres, usf_cm):
    s = {
        "wp":        S.wp_idx,
        "name":      WP_NAMES[S.wp_idx - 1] if S.wp_idx > 0 else "start",
        "time_s":    round(t, 1),
        "x_m":       round(x, 3),
        "y_m":       round(y, 3),
        "lat":       round(5.6037 + y * 0.000009, 7),   # fake coords near Ghana
        "lon":       round(-0.1870 + x * 0.000009, 7),
        "moisture1": m1,
        "moisture2": m2,
        "soil_t1":   t1,
        "soil_t2":   t2,
        "air_temp":  air,
        "humidity":  hum,
        "pressure":  pres,
        "us_front_cm": round(usf_cm, 1),
    }
    S.samples.append(s)
    return s

# ── Main simulation loop ───────────────────────────────────────
forward(MAX_SPEED)   # start moving

print("═" * 56)
print("  🌱 AgroRover Webots Simulation")
print("  Boustrophedon survey: 9 waypoints across 5 crop rows")
print("  Sensors: GPS · Compass · IMU · Camera · 2×Ultrasonic")
print("  + Simulated: Soil Moisture · Soil Temp · BME280")
print("═" * 56)

while robot.step(TIME_STEP) != -1:
    S.step_count += 1
    t = robot.getTime()

    # Read all hardware sensors
    gps_v   = gps.getValues()              # [x, y, z]
    comp_v  = comp.getValues()             # compass vector
    imu_v   = imu.getRollPitchYaw()        # [roll, pitch, yaw]
    accel_v = accel.getValues()            # [ax, ay, az]
    gyro_v  = gyro.getValues()             # [gx, gy, gz]
    usf_raw = usf.getValue()               # mm
    usr_raw = usr.getValue()               # mm
    usf_m   = usf_raw / 1000.0            # → metres
    usr_m   = usr_raw / 1000.0

    x, y = gps_v[0], gps_v[1]

    # Simulated soil sensors (mirrors Arduino serial output)
    m1, m2    = sim_moisture(x, y, t)
    t1, t2    = sim_soil_temp(x, y, t)
    air, hum, pres = sim_bme280(t)

    # Odometer
    if S.last_xy is not None:
        dx = x - S.last_xy[0]
        dy = y - S.last_xy[1]
        S.dist_total += math.sqrt(dx*dx + dy*dy)
    S.last_xy = (x, y)

    # ── Obstacle detection ───────────────────────────────────
    if usf_m < OBSTACLE_DIST_M and not S.avoiding and S.wp_idx < len(TARGETS):
        S.avoiding   = True
        S.avoid_step = 0
        S.avoid_phase= "back"
        print(f"\n  ⚠️  t={t:.1f}s OBSTACLE {usf_m*100:.0f}cm — activating avoidance")

    # ── State machine ────────────────────────────────────────
    if S.estop:
        stop()

    elif S.avoiding:
        run_avoidance()

    elif S.wp_idx >= len(TARGETS):
        stop()
        if S.step_count % 200 == 0:
            print(f"\n  ✅ SURVEY COMPLETE — {len(S.samples)} samples collected")

    else:
        left, right, dist = compute_steering(gps_v, comp_v, TARGETS[S.wp_idx])
        set_speed(left, right)

        # Waypoint reached?
        if dist < DISTANCE_TOLERANCE:
            sample = record_sample(t, x, y, m1, m2, t1, t2, air, hum, pres, usf_m*100)
            suffixes = {0:"st", 1:"nd", 2:"rd"}
            sfx = suffixes.get(S.wp_idx, "th")
            print(f"\n  📍 t={t:.1f}s  {S.wp_idx+1}{sfx} waypoint reached — {WP_NAMES[S.wp_idx]}")
            print(f"     moisture={m1:.0f}%/{m2:.0f}%  soil={t1:.1f}°C  air={air:.1f}°C")
            S.wp_idx += 1

            if S.wp_idx >= len(TARGETS):
                stop()
                print(f"\n{'═'*56}")
                print(f"  ✅ FIELD SURVEY COMPLETE")
                print(f"  Total distance: {S.dist_total:.1f}m")
                print(f"  Samples: {len(S.samples)}")
                print(f"{'═'*56}")
                print("\n  SOIL MOISTURE MAP:")
                for s in S.samples:
                    bar = "█" * int(s["moisture1"] / 5)
                    print(f"  WP{s['wp']:1d} ({s['x_m']:+5.1f},{s['y_m']:+5.1f}) "
                          f"M={s['moisture1']:4.0f}% {bar}")

    # ── Console dashboard every 2 seconds ───────────────────
    if t - S.last_print >= 2.0:
        S.last_print = t
        print_dashboard(t, gps_v, comp_v, usf_m, m1, m2, t1, t2, air, hum, pres)

    # ── JSON broadcast every 1 second (mirrors serial_handler) ─
    if t - S.last_json >= 1.0:
        S.last_json = t
        heading_deg = math.degrees(math.atan2(comp_v[0], comp_v[1])) % 360
        packet = {
            "t":          round(t, 1),
            "mode":       "AVOID" if S.avoiding else ("DONE" if S.wp_idx >= len(TARGETS) else "NAV"),
            "wp":         min(S.wp_idx + 1, len(TARGETS)),
            "pos":        {"x": round(x, 2), "y": round(y, 2), "z": round(gps_v[2], 2)},
            "heading_deg":round(heading_deg, 1),
            "imu_rpy":    [round(v, 3) for v in imu_v],
            "accel":      [round(v, 2) for v in accel_v],
            "gyro":       [round(v, 3) for v in gyro_v],
            "moisture":   [m1, m2],
            "soil_temp":  [t1, t2],
            "air_temp":   air,
            "humidity":   hum,
            "pressure":   pres,
            "us_front_cm":round(usf_m * 100, 1),
            "us_rear_cm": round(usr_m * 100, 1),
            "estop":      S.estop,
            "obstacle":   S.avoiding,
            "odometer_m": round(S.dist_total, 1),
            "samples":    len(S.samples),
        }
        print(json.dumps(packet))
