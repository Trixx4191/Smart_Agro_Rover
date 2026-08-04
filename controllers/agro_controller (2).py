```python
# ── State ──────────────────────────────────────────────────────
class State:
    wp_idx       = 0
    avoiding     = False
    avoid_step   = 0
    avoid_phase  = "back"
    turn_dir     = "right"
    recovering   = False
    recover_step = 0
    estop        = False
    samples      = []
    step_count   = 0
    last_print   = -99.0
    last_json    = -99.0
    dist_total   = 0.0
    last_xy      = None

S = State()

# ── Turn direction helper ──────────────────────────────────────
def choose_turn_direction(usf_m, usr_m):
    if usr_m > usf_m:
        return "left"
    return "right"

# ── Improved Obstacle Avoidance ────────────────────────────────
def run_avoidance(usf_m):
    S.avoid_step += 1

    # ✅ EARLY EXIT if path is clear
    if usf_m > OBSTACLE_DIST_M * 1.5:
        S.avoiding = False
        S.recovering = True
        S.recover_step = 0
        S.avoid_phase = "back"
        S.avoid_step = 0
        return

    if S.avoid_phase == "back":
        backward()
        if S.avoid_step > 25:
            S.avoid_phase = "turn"
            S.avoid_step = 0

    elif S.avoid_phase == "turn":
        if S.turn_dir == "left":
            spin_left()
        else:
            spin_right()

        if S.avoid_step > 30:
            S.avoid_phase = "forward"
            S.avoid_step = 0

    elif S.avoid_phase == "forward":
        forward(MAX_SPEED * 0.5)

        if S.avoid_step > 25:
            S.avoiding = False
            S.recovering = True
            S.recover_step = 0
            S.avoid_phase = "back"
            S.avoid_step = 0


# ── MAIN LOOP PATCHES ──────────────────────────────────────────
# Replace ONLY the obstacle + state logic inside your loop with this:

    # ── Obstacle detection ───────────────────────────────────
    if (usf_m < OBSTACLE_DIST_M 
        and not S.avoiding 
        and not S.recovering 
        and S.wp_idx < len(TARGETS)):

        S.avoiding   = True
        S.avoid_step = 0
        S.avoid_phase = "back"
        S.turn_dir   = choose_turn_direction(usf_m, usr_m)

        print(f"\n  ⚠️  t={t:.1f}s OBSTACLE {usf_m*100:.0f}cm — avoiding ({S.turn_dir})")

    # ── State machine ────────────────────────────────────────
    if S.estop:
        stop()

    elif S.avoiding:
        run_avoidance(usf_m)

    elif S.recovering:
        forward(MAX_SPEED * 0.5)
        S.recover_step += 1

        if S.recover_step > 30:
            S.recovering = False

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
```
