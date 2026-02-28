"""
rover_simulator.py
──────────────────
Simulates the Arduino firmware's serial output so the Raspberry Pi
software can be developed and tested without physical hardware.

Run this in one terminal, then point the Pi scripts at this port.
It also accepts commands exactly as the real firmware would.

Usage:
    python rover_simulator.py

Output: simulated JSON sensor packets + command acknowledgements
"""

import json
import time
import random
import threading
import sys
import math

#  Simulated sensor state 
state = {
    "moisture":   [55, 62],
    "soil_temp":  [24.3, 23.8],
    "air_temp":   27.1,
    "humidity":   68.4,
    "pressure":   1011.2,
    "us_front":   150,
    "us_rear":    180,
    "enc":        [0, 0, 0, 0],
    "estop":      False,
    "dir":        "stop",
    "speed":      0,
}

OBSTACLE_DIST = 20   # cm
SAMPLE_INTERVAL = 2.0  # seconds
running = True


def drift(val, amount=1.5, lo=0, hi=100):
    """Simulate natural sensor drift."""
    return round(min(hi, max(lo, val + random.uniform(-amount, amount))), 1)


def simulate_sensors():
    """Update simulated sensor values over time."""
    t = 0
    while running:
        t += 0.1
        state["moisture"][0] = drift(state["moisture"][0], 0.8, 0, 100)
        state["moisture"][1] = drift(state["moisture"][1], 0.8, 0, 100)
        state["soil_temp"][0] = round(drift(state["soil_temp"][0], 0.05, 10, 45), 1)
        state["soil_temp"][1] = round(drift(state["soil_temp"][1], 0.05, 10, 45), 1)
        state["air_temp"]  = round(drift(state["air_temp"],  0.03, 10, 50), 1)
        state["humidity"]  = round(drift(state["humidity"],  0.2,  10, 100), 1)
        state["pressure"]  = round(drift(state["pressure"],  0.1,  900, 1100), 1)

        # Simulate encoder ticks while moving
        if state["dir"] != "stop" and not state["estop"]:
            ticks = int(state["speed"] / 50)
            state["enc"] = [e + ticks for e in state["enc"]]

        # Simulate random obstacle appearing
        if state["dir"] == "forward":
            if random.random() < 0.01:  # 1% chance per tick
                state["us_front"] = random.randint(10, 19)
            else:
                state["us_front"] = random.randint(80, 250)
        else:
            state["us_front"] = random.randint(80, 250)

        state["us_rear"] = random.randint(80, 250)

        # Auto-stop on obstacle
        if state["us_front"] < OBSTACLE_DIST and not state["estop"]:
            if state["dir"] == "forward":
                state["dir"] = "stop"
                state["speed"] = 0
                alert = {"alert": "obstacle", "dist_cm": state["us_front"]}
                print(f"[SIM→PI] {json.dumps(alert)}", flush=True)

        time.sleep(0.1)


def broadcast_loop():
    """Emit sensor packet every SAMPLE_INTERVAL seconds."""
    while running:
        packet = {
            "moisture":  [int(state["moisture"][0]), int(state["moisture"][1])],
            "soil_temp": [state["soil_temp"][0], state["soil_temp"][1]],
            "air_temp":  state["air_temp"],
            "humidity":  state["humidity"],
            "pressure":  state["pressure"],
            "us_front":  state["us_front"],
            "us_rear":   state["us_rear"],
            "enc":       list(state["enc"]),
            "estop":     state["estop"],
            "dir":       state["dir"],
            "speed":     state["speed"],
        }
        print(f"[SIM→PI] {json.dumps(packet)}", flush=True)
        time.sleep(SAMPLE_INTERVAL)


def handle_command(raw: str):
    """Process a JSON command string, update state, return ack."""
    raw = raw.strip()
    if not raw:
        return None
    try:
        cmd_obj = json.loads(raw)
    except json.JSONDecodeError:
        return json.dumps({"error": "bad_json"})

    cmd = cmd_obj.get("cmd")
    if not cmd:
        return json.dumps({"error": "no_cmd"})

    if cmd == "estop":
        state["estop"] = True
        state["dir"] = "stop"
        state["speed"] = 0
        return json.dumps({"ack": "estop", "locked": True})

    if cmd == "reset":
        state["estop"] = False
        return json.dumps({"ack": "reset", "locked": False})

    if state["estop"]:
        return json.dumps({"error": "estop_active"})

    if cmd == "stop":
        state["dir"] = "stop"
        state["speed"] = 0
        return json.dumps({"ack": "stop"})

    if cmd == "move":
        direction = cmd_obj.get("dir", "forward")
        speed = int(cmd_obj.get("speed", 150))
        speed = max(0, min(255, speed))
        valid = ["forward", "backward", "left", "right"]
        if direction not in valid:
            return json.dumps({"error": "unknown_dir"})
        state["dir"] = direction
        state["speed"] = speed
        return json.dumps({"ack": "move", "dir": direction, "speed": speed})

    if cmd == "sample":
        packet = {
            "moisture":  [int(state["moisture"][0]), int(state["moisture"][1])],
            "soil_temp": [state["soil_temp"][0], state["soil_temp"][1]],
            "air_temp":  state["air_temp"],
            "humidity":  state["humidity"],
            "pressure":  state["pressure"],
            "us_front":  state["us_front"],
            "us_rear":   state["us_rear"],
            "enc":       list(state["enc"]),
            "estop":     state["estop"],
            "dir":       state["dir"],
            "speed":     state["speed"],
        }
        return json.dumps(packet)

    return json.dumps({"error": "unknown_cmd"})


def command_loop():
    """Read commands from stdin (simulating Pi serial writes)."""
    print("[SIM→PI] {\"status\":\"ready\",\"fw\":\"1.0.0\"}", flush=True)
    while running:
        try:
            line = input()
            response = handle_command(line)
            if response:
                print(f"[SIM→PI] {response}", flush=True)
        except EOFError:
            break


# ── Entry point ───────────────────────────────────────────────
if __name__ == "__main__":
    print("=" * 60)
    print("  FARM ROVER — Arduino Simulator")
    print("  Type JSON commands, see simulated responses")
    print("  Examples:")
    print('    {"cmd":"move","dir":"forward","speed":150}')
    print('    {"cmd":"stop"}')
    print('    {"cmd":"sample"}')
    print('    {"cmd":"estop"}')
    print('    {"cmd":"reset"}')
    print("=" * 60)

    sensor_thread    = threading.Thread(target=simulate_sensors, daemon=True)
    broadcast_thread = threading.Thread(target=broadcast_loop,   daemon=True)

    sensor_thread.start()
    broadcast_thread.start()

    try:
        command_loop()
    except KeyboardInterrupt:
        running = False
        print("\n[SIM] Stopped.")
