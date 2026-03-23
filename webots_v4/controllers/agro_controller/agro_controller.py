"""
AgroRover Controller
====================
Mirrors the sample moose_path_following.c controller exactly,
converted to Python with our farm rover sensor suite added.

Motor names (from Moose PROTO):
  left motor 1, left motor 2, left motor 3, left motor 4
  right motor 1, right motor 2, right motor 3, right motor 4
"""

from controller import Robot
import math
import json

# ── Constants (same as sample C controller) ──────────────────
TIME_STEP          = 16
MAX_SPEED          = 7.0
TURN_COEFFICIENT   = 4.0
DISTANCE_TOLERANCE = 1.5
OBSTACLE_DIST_M    = 1.2    # metres — stop if closer

# ── Farm survey waypoints (x, z coords in Webots) ────────────
# Same coordinate pattern as the sample
TARGETS = [
    (-4.209318,  9.147717),   # WP1 — field north-west
    ( 0.946812,  9.404304),   # WP2 — field north-east
    ( 0.175989, -1.784311),   # WP3 — field centre
    (-2.805353, -8.829694),   # WP4 — field south
    (-4.209318,  9.147717),   # WP5 — return to start
]

# ── Robot init ───────────────────────────────────────────────
robot = Robot()

# Motors — exact names from Moose PROTO
motor_names = [
    "left motor 1", "left motor 2", "left motor 3", "left motor 4",
    "right motor 1", "right motor 2", "right motor 3", "right motor 4",
]
motors = []
for name in motor_names:
    m = robot.getDevice(name)
    m.setPosition(float('inf'))   # velocity mode
    m.setVelocity(0.0)
    motors.append(m)

# Sensors
gps     = robot.getDevice("gps")
compass = robot.getDevice("compass")
us_front = robot.getDevice("us_front")
us_rear  = robot.getDevice("us_rear")

gps.enable(TIME_STEP)
compass.enable(TIME_STEP)
us_front.enable(TIME_STEP)
us_rear.enable(TIME_STEP)

# ── Helpers ───────────────────────────────────────────────────
def set_speed(left, right):
    for i in range(4):
        motors[i].setVelocity(left)      # left motors 1-4
        motors[i + 4].setVelocity(right) # right motors 1-4

def modulus(a, m):
    r = a - int(a / m) * m
    if r < 0:
        r += m
    return r

def get_simulated_moisture(x, z):
    """Simulate moisture — higher near crop rows at x = -8,-4,0,4,8"""
    nearest = round(x / 4) * 4
    dist    = abs(x - nearest)
    val     = max(10, min(95, 70 - dist * 8))
    return round(val, 1), round(val - 5 + (z % 3), 1)

def get_simulated_soil_temp(x, z):
    dist_c = math.sqrt(x*x + z*z)
    t1 = round(24.5 - dist_c * 0.05, 1)
    t2 = round(23.8 - dist_c * 0.04, 1)
    return t1, t2

# ── State ────────────────────────────────────────────────────
current_target = 0
avoiding        = False
avoid_timer     = 0
samples         = []
step            = 0
last_print      = 0

# Startup
set_speed(MAX_SPEED, MAX_SPEED)
print("=" * 55)
print("  AgroRover Webots Simulation")
print("  Waypoints:", len(TARGETS))
print("  Motors: 8 (4L + 4R) — Moose 4WD chassis")
print("=" * 55)

# ── Main loop ─────────────────────────────────────────────────
while robot.step(TIME_STEP) != -1:
    step += 1
    t = robot.getTime()

    # Read sensors
    pos      = gps.getValues()          # [x, y, z]
    north    = compass.getValues()      # [x, y, z] pointing north
    dist_f   = us_front.getValue() / 1000.0   # mm → m
    dist_r   = us_rear.getValue()  / 1000.0

    x, z = pos[0], pos[1]   # Webots NUE: x=east, y=north, z=up

    # Simulated soil sensors
    m1, m2   = get_simulated_moisture(x, z)
    t1, t2   = get_simulated_soil_temp(x, z)
    air_temp = round(28.5 + math.sin(t * 0.01) * 0.5, 1)
    humidity = round(65.0 + math.cos(t * 0.008) * 2.0, 1)
    pressure = round(1012.0 - abs(z) * 0.01, 1)

    # ── Obstacle check ────────────────────────────────────────
    if dist_f < OBSTACLE_DIST_M and not avoiding:
        avoiding    = True
        avoid_timer = 0
        print(f"\n  OBSTACLE {dist_f*100:.0f}cm — avoiding")

    if avoiding:
        avoid_timer += 1
        if avoid_timer < 30:
            set_speed(-MAX_SPEED * 0.6, -MAX_SPEED * 0.6)   # back up
        elif avoid_timer < 60:
            set_speed(-MAX_SPEED * 0.5, MAX_SPEED * 0.5)    # turn right
        else:
            avoiding    = False
            avoid_timer = 0
    else:
        # ── Waypoint navigation (same algorithm as sample C) ──
        if current_target < len(TARGETS):
            tx, tz = TARGETS[current_target]

            # Direction vector to target
            du = tx - x
            dv = tz - z
            distance = math.sqrt(du*du + dv*dv)

            # Waypoint reached?
            if distance < DISTANCE_TOLERANCE:
                # Record soil sample
                sample = {
                    "wp":        current_target + 1,
                    "time":      round(t, 1),
                    "x":         round(x, 2),
                    "z":         round(z, 2),
                    "moisture":  [m1, m2],
                    "soil_temp": [t1, t2],
                    "air_temp":  air_temp,
                    "humidity":  humidity,
                    "pressure":  pressure,
                }
                samples.append(sample)
                suffixes = ["st","nd","rd","th","th"]
                sfx = suffixes[min(current_target, 4)]
                print(f"\n  {current_target+1}{sfx} waypoint reached — sample recorded")
                print(f"     moisture={m1}% | soil_temp={t1}C | air={air_temp}C")
                current_target += 1

                if current_target >= len(TARGETS):
                    set_speed(0, 0)
                    print("\n  SURVEY COMPLETE")
                    print(f"  Total samples: {len(samples)}")
                    for s in samples:
                        print(f"  WP{s['wp']}: moisture={s['moisture']} temp={s['soil_temp']}")
            else:
                # Compute steering (same as sample C)
                robot_angle  = math.atan2(north[0], north[1])
                target_angle = math.atan2(dv, du)
                beta = modulus(target_angle - robot_angle, 2.0 * math.pi) - math.pi
                if beta > 0:
                    beta = math.pi - beta
                else:
                    beta = -beta - math.pi

                left_speed  = MAX_SPEED - math.pi + TURN_COEFFICIENT * beta
                right_speed = MAX_SPEED - math.pi - TURN_COEFFICIENT * beta
                set_speed(left_speed, right_speed)

    # ── Console dashboard every 2 seconds ────────────────────
    if t - last_print >= 2.0:
        last_print = t
        wp = current_target + 1
        status = "AVOIDING" if avoiding else "NAVIGATING"
        heading = math.degrees(math.atan2(north[0], north[1]))
        dirs = ["N","NE","E","SE","S","SW","W","NW","N"]
        dir_label = dirs[round((heading % 360) / 45) % 8]

        print(f"\n{'='*52}")
        print(f"  AGROROVER  t={t:.1f}s  [{status}]")
        print(f"  Pos: x={x:+.2f}  z={z:+.2f}  Heading: {heading:.0f}° {dir_label}")
        print(f"  Waypoint: {min(wp, len(TARGETS))}/{len(TARGETS)}  Front: {dist_f*100:.0f}cm")
        print(f"  Moisture: {m1}% | {m2}%    Soil T: {t1}°C | {t2}°C")
        print(f"  Air: {air_temp}°C  Humidity: {humidity}%  Pressure: {pressure}hPa")
        print(f"{'='*52}")

        # JSON packet (mirrors real serial_handler.py output)
        packet = {
            "t": round(t, 1),
            "mode": status,
            "pos": {"x": round(x,2), "z": round(z,2)},
            "waypoint": min(wp, len(TARGETS)),
            "moisture": [m1, m2],
            "soil_temp": [t1, t2],
            "air_temp": air_temp,
            "humidity": humidity,
            "pressure": pressure,
            "us_front_cm": round(dist_f*100, 1),
            "us_rear_cm":  round(dist_r*100, 1),
            "samples": len(samples),
        }
        print(json.dumps(packet))
