"""
AgroRover — Webots Farm Simulation Controller
==============================================
v1.2 — Navigation overhaul

ROOT CAUSE OF CRASHES (diagnosed):
  1. NO SPEED REDUCTION ON APPROACH: Rover hit row-end at full speed
     (7.0 rad/s), overshooting the waypoint by 1-2 m into the fence.
  2. TOLERANCE TOO LARGE (1.5 m): "reached" fired while still 1.5 m
     away and moving fast — rover never actually stopped.
  3. NO STOP-AND-ALIGN: After WP2→WP3 the required turn is 90°.
     compute_steering with 90° error clamps one motor to -MAX_SPEED,
     causing a violent spin that bounced the rover into crop rows.

FIX — Three-phase navigation per waypoint leg:
  ALIGN → rover stops and spins in-place until heading error < 8°
  DRIVE → full CRUISE_SPEED with differential steering
  BRAKE → ramps down to BRAKE_SPEED inside 4.5 m of waypoint

  On reaching the waypoint: stop, record sample, then immediately
  enter ALIGN toward the NEXT waypoint before driving again.
  Obstacle detection is suppressed during ALIGN (spinning generates
  false positives from nearby plants seen at angle).
"""

from controller import Robot
import math
import json

# ── Constants ─────────────────────────────────────────────────
TIME_STEP          = 16
MAX_SPEED          = 7.0      # rad/s  hard cap
CRUISE_SPEED       = 6.0      # rad/s  normal straight-line speed
BRAKE_SPEED        = 2.5      # rad/s  slow approach
ALIGN_SPEED        = 2.2      # rad/s  in-place spin
TURN_COEFFICIENT   = 4.0      # steering gain
DISTANCE_TOLERANCE = 0.7      # m  — tighter waypoint acceptance radius
BRAKE_DIST         = 4.5      # m  — begin braking this far from target
ALIGN_ANGLE_TOL    = math.radians(8)   # accept heading within 8°
OBSTACLE_DIST_M    = 1.2      # m  — front sonar trigger
REAR_CLEAR_M       = 0.6      # m  — rear guard during reverse

AVOID_BACK_T  = 1.8   # s
AVOID_TURN_T  = 2.4   # s
AVOID_FWD_T   = 1.4   # s

# ── Waypoints (x=east, y=north) ───────────────────────────────
# Pulled in to y=±19 (1 m gap from last plant at y=±18, 4 m from fence)
TARGETS = [
    (-6.0, -19.0),    # WP1  Row-A south
    (-6.0,  19.0),    # WP2  Row-A north
    (-2.0,  19.0),    # WP3  Row-B north
    (-2.0, -19.0),    # WP4  Row-B south
    ( 2.0, -19.0),    # WP5  Row-C south
    ( 2.0,  19.0),    # WP6  Row-C north
    ( 6.0,  19.0),    # WP7  Row-D north
    ( 6.0, -19.0),    # WP8  Row-D south
]
WP_NAMES = [
    "Row-A south", "Row-A north",
    "Row-B north", "Row-B south",
    "Row-C south", "Row-C north",
    "Row-D north", "Row-D south",
]
assert len(TARGETS) == len(WP_NAMES)

# ── Robot init ────────────────────────────────────────────────
robot = Robot()
motors = []
for mn in ["left motor 1","left motor 2","left motor 3","left motor 4",
           "right motor 1","right motor 2","right motor 3","right motor 4"]:
    m = robot.getDevice(mn)
    m.setPosition(float("inf"))
    m.setVelocity(0.0)
    motors.append(m)

gps   = robot.getDevice("gps");           gps.enable(TIME_STEP)
comp  = robot.getDevice("compass");       comp.enable(TIME_STEP)
imu   = robot.getDevice("imu");           imu.enable(TIME_STEP)
accel = robot.getDevice("accelerometer"); accel.enable(TIME_STEP)
gyro  = robot.getDevice("gyro");          gyro.enable(TIME_STEP)
cam   = robot.getDevice("camera");        cam.enable(TIME_STEP * 4)
usf   = robot.getDevice("us_front");      usf.enable(TIME_STEP)
usr   = robot.getDevice("us_rear");       usr.enable(TIME_STEP)

# ── Motor helpers ─────────────────────────────────────────────
def set_speed(left, right):
    left  = max(-MAX_SPEED, min(MAX_SPEED, left))
    right = max(-MAX_SPEED, min(MAX_SPEED, right))
    for i in range(4): motors[i].setVelocity(left)
    for i in range(4): motors[i+4].setVelocity(right)

def stop():                    set_speed(0.0, 0.0)
def backward(s=MAX_SPEED*0.5): set_speed(-s, -s)
def spin_left(s=ALIGN_SPEED):  set_speed(-s,  s)
def spin_right(s=ALIGN_SPEED): set_speed( s, -s)

# ── Navigation math ───────────────────────────────────────────
def modulus_double(a, m):
    r = a - int(a/m)*m
    if r < 0.0: r += m
    return r

def heading_error(comp_v, bearing):
    """Signed error in (-π, π]. +ve = turn left, -ve = turn right."""
    robot_h = math.atan2(comp_v[0], comp_v[1])
    return modulus_double(bearing - robot_h, 2*math.pi) - math.pi

def compute_steering(gps_v, comp_v, target, speed_cap=CRUISE_SPEED):
    du = target[0] - gps_v[0]
    dv = target[1] - gps_v[1]
    dist = math.sqrt(du*du + dv*dv)
    if dist < 1e-6:
        return 0.0, 0.0, 0.0
    bearing = math.atan2(dv, du)
    err     = heading_error(comp_v, bearing)
    base    = min(speed_cap, CRUISE_SPEED) - math.pi
    return base + TURN_COEFFICIENT*err, base - TURN_COEFFICIENT*err, dist

# ── Sensor simulation ─────────────────────────────────────────
def sim_moisture(x, y, t):
    d   = abs(x - round(x/4.0)*4.0)
    ch1 = max(5.0, min(98.0, 72.0-d*9.0 + 3.0*math.sin(t*0.3+y*0.1)))
    ch2 = max(5.0, min(98.0, 68.0-d*9.0 + 2.5*math.cos(t*0.25+x*0.1)))
    return round(ch1,1), round(ch2,1)

def sim_soil_temp(x, y, t):
    d = abs(x+20.0)
    return (round(24.5-d*0.05+0.3*math.sin(t*0.05),2),
            round(23.8-d*0.04+0.2*math.cos(t*0.07),2))

def sim_bme280(t):
    return (round(28.5+0.8*math.sin(t*0.02),1),
            round(64.0+3.0*math.cos(t*0.015),1),
            round(1012.0-0.5*abs(math.sin(t*0.01)),1))

# ── State ─────────────────────────────────────────────────────
class State:
    wp_idx      = 0
    nav_phase   = "align"   # "align" | "drive" | "brake"
    avoiding    = False
    avoid_phase = "idle"
    avoid_start = 0.0
    estop       = False
    samples     = []
    step_count  = 1
    last_print  = -99.0
    last_json   = -99.0
    dist_total  = 0.0
    last_xy     = None

S = State()

# ── Avoidance ─────────────────────────────────────────────────
def start_avoidance(t):
    S.avoiding    = True
    S.avoid_phase = "back"
    S.avoid_start = t
    S.nav_phase   = "align"   # re-align after recovery
    print(f"\n  ⚠️  t={t:.1f}s  OBSTACLE — avoidance start")

def _nap(t, ph):
    S.avoid_phase = ph; S.avoid_start = t

def run_avoidance(t, usr_m):
    e = t - S.avoid_start
    if S.avoid_phase == "back":
        if usr_m < REAR_CLEAR_M:
            print(f"  ⚠️  t={t:.1f}s  Rear blocked — skipping reverse")
            _nap(t, "turn")
        elif e < AVOID_BACK_T: backward()
        else: _nap(t, "turn")
    elif S.avoid_phase == "turn":
        if e < AVOID_TURN_T: spin_right()
        else: _nap(t, "fwd")
    elif S.avoid_phase == "fwd":
        if e < AVOID_FWD_T: set_speed(MAX_SPEED*0.7, MAX_SPEED*0.7)
        else:
            S.avoiding = False; S.avoid_phase = "idle"
            print(f"  ✅  t={t:.1f}s  Avoidance done — realigning to WP{S.wp_idx+1}")
            return True
    return False

# ── Navigation ────────────────────────────────────────────────
def run_navigation(t, gps_v, comp_v):
    """Returns True when current waypoint is reached."""
    target  = TARGETS[S.wp_idx]
    du      = target[0] - gps_v[0]
    dv      = target[1] - gps_v[1]
    dist    = math.sqrt(du*du + dv*dv)
    bearing = math.atan2(dv, du)
    err     = heading_error(comp_v, bearing)

    # ALIGN — stop and spin until facing the target
    if S.nav_phase == "align":
        if abs(err) > ALIGN_ANGLE_TOL:
            if err > 0: spin_left()
            else:       spin_right()
        else:
            S.nav_phase = "brake" if dist < BRAKE_DIST else "drive"
            print(f"  🧭 t={t:.1f}s  Aligned → {S.nav_phase.upper()}"
                  f"  WP{S.wp_idx+1}  err={math.degrees(err):.1f}°")
        return False

    # Transition: close enough to start braking?
    if S.nav_phase == "drive" and dist < BRAKE_DIST:
        S.nav_phase = "brake"

    # Speed cap for BRAKE: linear ramp from CRUISE down to BRAKE_SPEED
    if S.nav_phase == "brake":
        fraction  = max(0.0, dist / BRAKE_DIST)
        speed_cap = BRAKE_SPEED + (CRUISE_SPEED - BRAKE_SPEED) * fraction
    else:
        speed_cap = CRUISE_SPEED

    left, right, _ = compute_steering(gps_v, comp_v, target, speed_cap)
    set_speed(left, right)

    if dist < DISTANCE_TOLERANCE:
        stop()
        return True
    return False

# ── Helpers ───────────────────────────────────────────────────
def ordinal(n):
    if 11 <= (n%100) <= 13: return f"{n}th"
    return f"{n}{['th','st','nd','rd','th','th','th','th','th','th'][n%10]}"

COMPASS_LABELS = ["N","NNE","NE","ENE","E","ESE","SE","SSE",
                  "S","SSW","SW","WSW","W","WNW","NW","NNW"]

def print_dashboard(t, gps_v, comp_v, usf_m, m1, m2, t1, t2, air, hum, pres):
    x, y = gps_v[0], gps_v[1]
    hdeg = math.degrees(math.atan2(comp_v[0], comp_v[1])) % 360
    clbl = COMPASS_LABELS[round(hdeg/22.5) % 16]
    wpn  = WP_NAMES[S.wp_idx] if S.wp_idx < len(WP_NAMES) else "DONE"
    mode = ("AVOIDING" if S.avoiding else
            "COMPLETE" if S.wp_idx >= len(TARGETS) else
            S.nav_phase.upper())
    dist_wp = 0.0
    if S.wp_idx < len(TARGETS):
        tx,ty = TARGETS[S.wp_idx]
        dist_wp = math.sqrt((tx-x)**2+(ty-y)**2)
    print(f"\n{'═'*56}")
    print(f"  🌱 AGROROVER  t={t:.1f}s   MODE: {mode}")
    print(f"{'═'*56}")
    print(f"  📍 POS:      x={x:+6.2f}m  y={y:+6.2f}m  h={gps_v[2]:.2f}m")
    print(f"  🧭 HEADING:  {hdeg:5.1f}°  {clbl:<4}  Dist→WP: {dist_wp:.1f}m")
    print(f"  🎯 WAYPOINT: {S.wp_idx+1}/{len(TARGETS)} — {wpn}")
    print(f"  📡 OBSTACLE: front={usf_m*100:.0f}cm "
          f"{'⚠️  AVOID' if S.avoiding else '✅ clear'}")
    print(f"  💧 MOISTURE: ch1={m1:.0f}%   ch2={m2:.0f}%")
    print(f"  🌡  SOIL T:  {t1:.1f}°C  |  {t2:.1f}°C")
    print(f"  🌤  AIR:     {air:.1f}°C   {hum:.0f}%RH   {pres:.1f}hPa")
    print(f"  🛣  ODOMETER:{S.dist_total:.1f}m   Samples: {len(S.samples)}")
    if S.estop: print("  🛑 EMERGENCY STOP ACTIVE")
    print(f"{'═'*56}")

def record_sample(t, x, y, name, m1, m2, t1, t2, air, hum, pres, usf_cm):
    s = {"wp":S.wp_idx,"name":name,"time_s":round(t,1),
         "x_m":round(x,3),"y_m":round(y,3),
         "lat":round(5.6037+y*0.000009,7),"lon":round(-0.1870+x*0.000009,7),
         "moisture1":m1,"moisture2":m2,"soil_t1":t1,"soil_t2":t2,
         "air_temp":air,"humidity":hum,"pressure":pres,
         "us_front_cm":round(usf_cm,1)}
    S.samples.append(s)
    return s

def find_nearest_wp(x, y):
    bi, bd = 0, float("inf")
    for i,(tx,ty) in enumerate(TARGETS):
        d = math.sqrt((tx-x)**2+(ty-y)**2)
        if d < bd: bd,bi = d,i
    return bi, bd

# ── Boot: one step for valid GPS, then align ─────────────────
robot.step(TIME_STEP)
_g = gps.getValues()
S.wp_idx, _sd = find_nearest_wp(_g[0], _g[1])
S.last_xy     = (_g[0], _g[1])
S.nav_phase   = "align"

print("═"*56)
print("  🌱 AgroRover Webots Simulation  v1.2")
print("  Nav: ALIGN → DRIVE → BRAKE(4.5m) → reached → ALIGN")
print("═"*56)
print(f"  📍 Spawn: x={_g[0]:+.2f}m  y={_g[1]:+.2f}m")
print(f"  🎯 Nearest WP: {S.wp_idx+1} — {WP_NAMES[S.wp_idx]}  ({_sd:.1f} m)")
print("═"*56)

# ── Main loop ─────────────────────────────────────────────────
while robot.step(TIME_STEP) != -1:
    S.step_count += 1
    t = robot.getTime()

    gps_v   = gps.getValues()
    comp_v  = comp.getValues()
    imu_v   = imu.getRollPitchYaw()
    accel_v = accel.getValues()
    gyro_v  = gyro.getValues()
    usf_m   = usf.getValue() / 1000.0
    usr_m   = usr.getValue() / 1000.0
    x, y    = gps_v[0], gps_v[1]

    m1, m2         = sim_moisture(x, y, t)
    t1, t2         = sim_soil_temp(x, y, t)
    air, hum, pres = sim_bme280(t)

    if S.last_xy:
        dx=x-S.last_xy[0]; dy=y-S.last_xy[1]
        S.dist_total += math.sqrt(dx*dx+dy*dy)
    S.last_xy = (x, y)

    # Obstacle check — suppressed during ALIGN to avoid plant false-positives
    if (usf_m < OBSTACLE_DIST_M
            and not S.avoiding and not S.estop
            and S.nav_phase != "align"
            and S.wp_idx < len(TARGETS)):
        start_avoidance(t)

    # Control
    if S.estop:
        stop()
    elif S.avoiding:
        run_avoidance(t, usr_m)
    elif S.wp_idx >= len(TARGETS):
        stop()
        if S.step_count % 200 == 0:
            print(f"\n  ✅ SURVEY COMPLETE — {len(S.samples)} samples")
    else:
        if run_navigation(t, gps_v, comp_v):
            name = WP_NAMES[S.wp_idx]
            record_sample(t,x,y,name,m1,m2,t1,t2,air,hum,pres,usf_m*100)
            print(f"\n  📍 t={t:.1f}s  {ordinal(S.wp_idx+1)} WP — {name}")
            print(f"     M={m1:.0f}%/{m2:.0f}%  soil={t1:.1f}°C  air={air:.1f}°C")
            S.wp_idx += 1
            if S.wp_idx >= len(TARGETS):
                stop()
                print(f"\n{'═'*56}\n  ✅ FIELD SURVEY COMPLETE")
                print(f"  Distance: {S.dist_total:.1f} m   Samples: {len(S.samples)}")
                print(f"{'═'*56}\n\n  SOIL MOISTURE MAP:")
                for s in S.samples:
                    print(f"  WP{s['wp']+1} ({s['x_m']:+5.1f},{s['y_m']:+5.1f})"
                          f"  {'█'*int(s['moisture1']/5)}  {s['moisture1']:.0f}%")
            else:
                S.nav_phase = "align"   # face next WP before driving

    if t - S.last_print >= 2.0:
        S.last_print = t
        print_dashboard(t,gps_v,comp_v,usf_m,m1,m2,t1,t2,air,hum,pres)

    if t - S.last_json >= 1.0:
        S.last_json = t
        hdeg = math.degrees(math.atan2(comp_v[0], comp_v[1])) % 360
        print(json.dumps({
            "t":round(t,1),
            "mode":("AVOID" if S.avoiding else
                    "DONE"  if S.wp_idx>=len(TARGETS) else
                    S.nav_phase.upper()),
            "wp":min(S.wp_idx+1,len(TARGETS)),
            "nav_phase":S.nav_phase,
            "pos":{"x":round(x,2),"y":round(y,2),"z":round(gps_v[2],2)},
            "heading_deg":round(hdeg,1),
            "imu_rpy":[round(v,3) for v in imu_v],
            "accel":[round(v,2) for v in accel_v],
            "gyro":[round(v,3) for v in gyro_v],
            "moisture":[m1,m2],"soil_temp":[t1,t2],
            "air_temp":air,"humidity":hum,"pressure":pres,
            "us_front_cm":round(usf_m*100,1),
            "us_rear_cm":round(usr_m*100,1),
            "estop":S.estop,"obstacle":S.avoiding,
            "avoid_phase":S.avoid_phase,
            "odometer_m":round(S.dist_total,1),
            "samples":len(S.samples),
        }))
