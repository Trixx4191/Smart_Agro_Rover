
from controller import Robot
import math
import json

# ══════════════════════════════════════════════════════════════
#  AgroRover Webots Controller  v2.0  — Clean Boustrophedon
#  Changes from v1.1:
#   • Obstacle avoidance removed (boxes removed from scene)
#   • Headland waypoints added at y=±22 so ALL lateral row-change
#     moves happen OUTSIDE the plant zone — rover never cuts
#     through a crop row during a turn
#   • TURN_COEFFICIENT lowered 4.0 → 2.2  (smoother arcs, no overshoot)
#   • Approach-speed reduction: slows to 40% within 3 m of waypoint
#     so it arrives cleanly and doesn't swing past the target
#   • Straight-line boost: full speed when heading error < 5°
# ══════════════════════════════════════════════════════════════

# ── Timing ────────────────────────────────────────────────────
TIME_STEP        = 16       # ms — matches WorldInfo basicTimeStep

# ── Speed ─────────────────────────────────────────────────────
MAX_SPEED        = 7.0      # rad/s  full forward speed
APPROACH_SPEED   = MAX_SPEED * 0.40   # slow speed near waypoint
APPROACH_DIST    = 3.0      # m  — start slowing within this radius

# ── Navigation ────────────────────────────────────────────────
TURN_COEFFICIENT  = 2.2     # steering gain  (was 4.0 — reduced to stop overshoot)
DIST_TOLERANCE    = 1.2     # m  — waypoint reached threshold

# ── Sample broadcast ──────────────────────────────────────────
SAMPLE_INTERVAL   = 3.0     # s

# ══════════════════════════════════════════════════════════════
#  BOUSTROPHEDON WAYPOINTS  — with headland clearance
#
#  Field:  x = ±10 m (E/W),  y = ±20 m (N/S)
#  Crop rows (Cypress trees) at x = -8, -4, 0, +4, +8
#  Rover lanes between rows  at x = -6, -2, +2, +6
#
#  KEY FIX: all lateral moves between rows happen at y = ±22 m
#  (2 m outside the plant zone).  The rover exits the field,
#  moves sideways in clear headland, then re-enters the next lane.
#  This prevents any possibility of swinging into a crop row
#  during a turn, no matter what the steering gain does.
#
#  Sequence (20 waypoints):
#
#    WP1  Lane-A south entry       (-6, -20)
#    WP2  Lane-A north exit        (-6, +22)  ← past plant zone
#    WP3  Lane-B north headland    (-2, +22)  ← lateral in headland
#    WP4  Lane-B south exit        (-2, -22)  ← past plant zone
#    WP5  Lane-C south headland    (+2, -22)  ← lateral in headland
#    WP6  Lane-C north exit        (+2, +22)  ← past plant zone
#    WP7  Lane-D north headland    (+6, +22)  ← lateral in headland
#    WP8  Lane-D south end         (+6, -20)
# ══════════════════════════════════════════════════════════════

HL = 22.0   # headland y-coordinate (outside ±20 m plant zone)

TARGETS = [
    (-6.0, -20.0),   # WP1  Lane-A south entry
    (-6.0,  HL  ),   # WP2  Lane-A north exit (headland)
    (-2.0,  HL  ),   # WP3  Lateral → Lane-B  (headland)
    (-2.0, -HL  ),   # WP4  Lane-B south exit (headland)
    ( 2.0, -HL  ),   # WP5  Lateral → Lane-C  (headland)
    ( 2.0,  HL  ),   # WP6  Lane-C north exit (headland)
    ( 6.0,  HL  ),   # WP7  Lateral → Lane-D  (headland)
    ( 6.0, -20.0),   # WP8  Lane-D south end
]

WP_NAMES = [
    "Lane-A south entry",
    "Lane-A north exit",
    "Lane-B headland north",
    "Lane-B south exit",
    "Lane-C headland south",
    "Lane-C north exit",
    "Lane-D headland north",
    "Lane-D south end",
]

# Which waypoints are lateral headland moves (no sensors needed, pure transit)
# These are transitions between lanes: WP3 and WP5 and WP7
HEADLAND_WPS = {2, 4, 6}   # 0-indexed: WP3, WP5, WP7

assert len(TARGETS) == len(WP_NAMES)

# ── Robot init ────────────────────────────────────────────────
robot = Robot()

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
gps   = robot.getDevice("gps");           gps.enable(TIME_STEP)
comp  = robot.getDevice("compass");       comp.enable(TIME_STEP)
imu   = robot.getDevice("imu");           imu.enable(TIME_STEP)
accel = robot.getDevice("accelerometer"); accel.enable(TIME_STEP)
gyro  = robot.getDevice("gyro");          gyro.enable(TIME_STEP)
cam   = robot.getDevice("camera");        cam.enable(TIME_STEP * 4)
usf   = robot.getDevice("us_front");      usf.enable(TIME_STEP)
usr   = robot.getDevice("us_rear");       usr.enable(TIME_STEP)

# ── Motor helpers ─────────────────────────────────────────────
def set_speed(left: float, right: float):
    left  = max(-MAX_SPEED, min(MAX_SPEED, left))
    right = max(-MAX_SPEED, min(MAX_SPEED, right))
    for i in range(4):
        motors[i    ].setVelocity(left)
        motors[i + 4].setVelocity(right)

def stop():
    set_speed(0.0, 0.0)

# ── Navigation math ───────────────────────────────────────────
def modulus_double(a: float, m: float) -> float:
    r = a - int(a / m) * m
    if r < 0.0:
        r += m
    return r

def compute_steering(gps_vals, compass_vals, target, dist: float):
    """
    Returns (left_speed, right_speed).

    Improvements over v1.1:
    • Uses the caller-supplied distance so we don't recalculate it.
    • Scales effective turn coefficient by heading error:
        - heading error < 5°  → nearly zero correction, full speed ahead
        - heading error > 30° → full TURN_COEFFICIENT correction
      This prevents the rover from weaving along a straight row.
    • Caps speed to APPROACH_SPEED when within APPROACH_DIST of target
      so the rover arrives gently and doesn't swing past.
    """
    x, y   = gps_vals[0], gps_vals[1]
    tx, ty = target

    du, dv = tx - x, ty - y
    if dist < 1e-6:
        return 0.0, 0.0

    robot_angle  = math.atan2(compass_vals[0], compass_vals[1])
    target_angle = math.atan2(dv, du)
    beta = modulus_double(target_angle - robot_angle, 2.0 * math.pi) - math.pi

    if beta > 0:
        beta =  math.pi - beta
    else:
        beta = -math.pi - beta

    # Scale down steering when almost straight — prevents row wobble
    heading_err_deg = abs(math.degrees(beta))
    if heading_err_deg < 5.0:
        tc = TURN_COEFFICIENT * 0.05    # virtually zero correction
    elif heading_err_deg < 30.0:
        tc = TURN_COEFFICIENT * (heading_err_deg / 30.0)
    else:
        tc = TURN_COEFFICIENT

    # Approach speed cap — slow down near waypoint to avoid overshoot
    spd = MAX_SPEED if dist > APPROACH_DIST else APPROACH_SPEED

    # Base formula (same as moose_path_following.c)
    base  = spd - math.pi
    left  = base + tc * beta
    right = base - tc * beta

    # Clamp to approach speed when near waypoint
    left  = max(-spd, min(spd, left))
    right = max(-spd, min(spd, right))

    return left, right

# ── Simulated soil / env sensors ─────────────────────────────
def sim_moisture(x: float, y: float, t: float):
    nearest     = round(x / 4.0) * 4.0
    dist_to_row = abs(x - nearest)
    base   = 72.0 - dist_to_row * 9.0
    noise1 = 3.0 * math.sin(t * 0.3  + y * 0.1)
    noise2 = 2.5 * math.cos(t * 0.25 + x * 0.1)
    ch1 = max(5.0, min(98.0, base + noise1))
    ch2 = max(5.0, min(98.0, base - 4.0 + noise2))
    return round(ch1, 1), round(ch2, 1)

def sim_soil_temp(x: float, y: float, t: float):
    dist_water = abs(x + 20.0)
    t1 = 24.5 - dist_water * 0.05 + 0.3 * math.sin(t * 0.05)
    t2 = 23.8 - dist_water * 0.04 + 0.2 * math.cos(t * 0.07)
    return round(t1, 2), round(t2, 2)

def sim_bme280(t: float):
    air  = 28.5  + 0.8 * math.sin(t * 0.02)
    hum  = 64.0  + 3.0 * math.cos(t * 0.015)
    pres = 1012.0 - 0.5 * abs(math.sin(t * 0.01))
    return round(air, 1), round(hum, 1), round(pres, 1)

# ── State ─────────────────────────────────────────────────────
class State:
    wp_idx      = 0
    estop       = False
    samples     = []
    step_count  = 1
    last_print  = -99.0
    last_json   = -99.0
    dist_total  = 0.0
    last_xy     = None

S = State()

# ── Helpers ───────────────────────────────────────────────────
def ordinal(n: int) -> str:
    if 11 <= (n % 100) <= 13:
        return f"{n}th"
    return f"{n}{['th','st','nd','rd','th','th','th','th','th','th'][n % 10]}"

COMPASS_LABELS = ["N","NNE","NE","ENE","E","ESE","SE","SSE",
                  "S","SSW","SW","WSW","W","WNW","NW","NNW"]

# ── Console dashboard ─────────────────────────────────────────
def print_dashboard(t, gps_v, comp_v, usf_m, m1, m2, t1, t2, air, hum, pres):
    x, y = gps_v[0], gps_v[1]
    heading_deg = math.degrees(math.atan2(comp_v[0], comp_v[1])) % 360
    compass_lbl = COMPASS_LABELS[round(heading_deg / 22.5) % 16]

    wp_num  = S.wp_idx + 1
    wp_name = WP_NAMES[S.wp_idx] if S.wp_idx < len(WP_NAMES) else "DONE"
    mode    = "COMPLETE" if S.wp_idx >= len(TARGETS) else "SURVEY"
    phase   = "(headland transit)" if S.wp_idx in HEADLAND_WPS else "(row survey)"

    dist_wp = 0.0
    if S.wp_idx < len(TARGETS):
        tx, ty  = TARGETS[S.wp_idx]
        dist_wp = math.sqrt((tx - x) ** 2 + (ty - y) ** 2)

    print(f"\n{'═'*58}")
    print(f"  🌱 AGROROVER v2.0   t={t:.1f}s   MODE: {mode}  {phase}")
    print(f"{'═'*58}")
    print(f"  📍 POS:      x={x:+6.2f}m  y={y:+6.2f}m  h={gps_v[2]:.2f}m")
    print(f"  🧭 HEADING:  {heading_deg:5.1f}°  {compass_lbl:<4}  Dist→WP: {dist_wp:.1f}m")
    print(f"  🎯 WAYPOINT: {wp_num}/{len(TARGETS)} — {wp_name}")
    print(f"  📡 FRONT US: {usf_m*100:.0f} cm")
    print(f"  💧 MOISTURE: ch1={m1:.0f}%   ch2={m2:.0f}%")
    print(f"  🌡  SOIL T:  {t1:.1f}°C  |  {t2:.1f}°C")
    print(f"  🌤  AIR:     {air:.1f}°C   {hum:.0f}%RH   {pres:.1f}hPa")
    print(f"  🛣  ODOMETER:{S.dist_total:.1f}m   Samples: {len(S.samples)}")
    if S.estop:
        print("  🛑 EMERGENCY STOP ACTIVE")
    print(f"{'═'*58}")

# ── Sample recording ──────────────────────────────────────────
def record_sample(t, x, y, wp_name, m1, m2, t1, t2, air, hum, pres, usf_cm):
    """Only called for actual survey waypoints, not headland transits."""
    s = {
        "wp":          S.wp_idx,
        "name":        wp_name,
        "time_s":      round(t, 1),
        "x_m":         round(x, 3),
        "y_m":         round(y, 3),
        "lat":         round(5.6037 + y * 0.000009, 7),
        "lon":         round(-0.1870 + x * 0.000009, 7),
        "moisture1":   m1,
        "moisture2":   m2,
        "soil_t1":     t1,
        "soil_t2":     t2,
        "air_temp":    air,
        "humidity":    hum,
        "pressure":    pres,
        "us_front_cm": round(usf_cm, 1),
    }
    S.samples.append(s)
    return s

# ── Find nearest waypoint on startup ─────────────────────────
def find_nearest_wp(x: float, y: float):
    best_idx, best_dist = 0, float("inf")
    for i, (tx, ty) in enumerate(TARGETS):
        d = math.sqrt((tx - x) ** 2 + (ty - y) ** 2)
        if d < best_dist:
            best_dist, best_idx = d, i
    return best_idx, best_dist

# ══════════════════════════════════════════════════════════════
#  STARTUP — one step to prime GPS, then find nearest waypoint
# ══════════════════════════════════════════════════════════════
robot.step(TIME_STEP)
_init_gps = gps.getValues()
S.wp_idx, _snap_dist = find_nearest_wp(_init_gps[0], _init_gps[1])
S.last_xy = (_init_gps[0], _init_gps[1])

set_speed(MAX_SPEED, MAX_SPEED)   # start moving

print("═" * 58)
print("  🌱 AgroRover Webots Simulation  v2.0  — Clean Boustrophedon")
print("  Path: 8 waypoints with headland clearance at y=±22 m")
print("  Lateral row changes happen OUTSIDE the plant zone")
print("  Obstacle avoidance: disabled (field cleared of boxes)")
print("═" * 58)
print(f"  📍 Spawn: x={_init_gps[0]:+.2f}m  y={_init_gps[1]:+.2f}m")
print(f"  🎯 Nearest WP: WP{S.wp_idx + 1} — {WP_NAMES[S.wp_idx]}"
      f"  ({_snap_dist:.1f} m away)")
print("═" * 58)
print()
print("  Waypoint sequence:")
for i, ((tx, ty), nm) in enumerate(zip(TARGETS, WP_NAMES)):
    tag = "  [headland]" if i in HEADLAND_WPS else ""
    print(f"    WP{i+1:1d}  ({tx:+5.1f}, {ty:+5.1f})  {nm}{tag}")
print("═" * 58)

# ══════════════════════════════════════════════════════════════
#  MAIN LOOP
# ══════════════════════════════════════════════════════════════
while robot.step(TIME_STEP) != -1:
    S.step_count += 1
    t = robot.getTime()

    # ── Read sensors ─────────────────────────────────────────
    gps_v   = gps.getValues()
    comp_v  = comp.getValues()
    imu_v   = imu.getRollPitchYaw()
    accel_v = accel.getValues()
    gyro_v  = gyro.getValues()
    usf_m   = usf.getValue() / 1000.0   # mm → m
    usr_m   = usr.getValue() / 1000.0

    x, y = gps_v[0], gps_v[1]

    # Simulated soil/env sensors
    m1, m2         = sim_moisture(x, y, t)
    t1, t2         = sim_soil_temp(x, y, t)
    air, hum, pres = sim_bme280(t)

    # Odometer
    if S.last_xy is not None:
        dx = x - S.last_xy[0]
        dy = y - S.last_xy[1]
        S.dist_total += math.sqrt(dx * dx + dy * dy)
    S.last_xy = (x, y)

    # ── Control ──────────────────────────────────────────────
    if S.estop:
        stop()

    elif S.wp_idx >= len(TARGETS):
        # Survey complete
        stop()
        if S.step_count % 200 == 0:
            print(f"\n  ✅ SURVEY COMPLETE — {len(S.samples)} samples collected"
                  f"  |  distance: {S.dist_total:.1f} m")

    else:
        target = TARGETS[S.wp_idx]
        tx, ty = target
        dist = math.sqrt((tx - x) ** 2 + (ty - y) ** 2)

        # Waypoint reached?
        if dist < DIST_TOLERANCE:
            current_name = WP_NAMES[S.wp_idx]
            in_headland  = (S.wp_idx in HEADLAND_WPS)

            if not in_headland:
                # Record soil data at survey waypoints only
                record_sample(t, x, y, current_name,
                              m1, m2, t1, t2, air, hum, pres, usf_m * 100)
                print(f"\n  📍 t={t:.1f}s  {ordinal(S.wp_idx + 1)} waypoint reached"
                      f" — {current_name}")
                print(f"     moisture={m1:.0f}%/{m2:.0f}%  soil={t1:.1f}°C"
                      f"  air={air:.1f}°C")
            else:
                print(f"\n  🔄 t={t:.1f}s  Headland turn — {current_name}")

            S.wp_idx += 1

            if S.wp_idx >= len(TARGETS):
                stop()
                print(f"\n{'═'*58}")
                print("  ✅ FIELD SURVEY COMPLETE")
                print(f"  Total distance: {S.dist_total:.1f} m")
                print(f"  Samples collected: {len(S.samples)}")
                print(f"{'═'*58}")
                print("\n  SOIL MOISTURE MAP:")
                for s in S.samples:
                    bar = "█" * int(s["moisture1"] / 5)
                    print(f"  WP{s['wp']+1} ({s['x_m']:+5.1f},{s['y_m']:+5.1f})"
                          f"  M={s['moisture1']:4.0f}%  {bar}")
        else:
            # Navigate toward current waypoint
            left, right = compute_steering(gps_v, comp_v, target, dist)
            set_speed(left, right)

    # ── Console dashboard every 2 s ──────────────────────────
    if t - S.last_print >= 2.0:
        S.last_print = t
        print_dashboard(t, gps_v, comp_v, usf_m,
                        m1, m2, t1, t2, air, hum, pres)

    # ── JSON telemetry every 1 s ─────────────────────────────
    if t - S.last_json >= 1.0:
        S.last_json = t
        heading_deg = math.degrees(math.atan2(comp_v[0], comp_v[1])) % 360
        packet = {
            "t":           round(t, 1),
            "mode":        "DONE" if S.wp_idx >= len(TARGETS) else (
                           "HEADLAND" if S.wp_idx in HEADLAND_WPS else "NAV"),
            "wp":          min(S.wp_idx + 1, len(TARGETS)),
            "wp_name":     WP_NAMES[S.wp_idx] if S.wp_idx < len(WP_NAMES) else "DONE",
            "pos":         {"x": round(x, 2), "y": round(y, 2),
                            "z": round(gps_v[2], 2)},
            "heading_deg": round(heading_deg, 1),
            "imu_rpy":     [round(v, 3) for v in imu_v],
            "accel":       [round(v, 2) for v in accel_v],
            "gyro":        [round(v, 3) for v in gyro_v],
            "moisture":    [m1, m2],
            "soil_temp":   [t1, t2],
            "air_temp":    air,
            "humidity":    hum,
            "pressure":    pres,
            "us_front_cm": round(usf_m * 100, 1),
            "us_rear_cm":  round(usr_m * 100, 1),
            "estop":       S.estop,
            "odometer_m":  round(S.dist_total, 1),
            "samples":     len(S.samples),
        }
        print(json.dumps(packet))
