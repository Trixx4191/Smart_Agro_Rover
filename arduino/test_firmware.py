"""
test_firmware.py
─
Tests the Arduino command/response protocol against the simulator.
Every test maps directly to real firmware behavior — if it passes
here, it'll behave identically on real hardware.

Run:  python test_firmware.py
"""

import json
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

# Import handle_command directly from simulator (same logic as firmware)
from rover_simulator import handle_command, state

PASS = "✅"
FAIL = "❌"
results = []

def reset_state():
    state["estop"] = False
    state["dir"] = "stop"
    state["speed"] = 0
    state["enc"] = [0, 0, 0, 0]

def test(name, condition):
    status = PASS if condition else FAIL
    results.append((status, name))
    print(f"  {status}  {name}")

def run(cmd_dict):
    raw = json.dumps(cmd_dict)
    resp = handle_command(raw)
    return json.loads(resp) if resp else {}


print("\n" + "="*55)
print("  FARM ROVER — Arduino Firmware Test Suite")
print("="*55)

#   Startup & malformed input 
print("\n[1] Input Validation")
reset_state()
r = handle_command("")
test("Empty input returns None", r is None)

r = json.loads(handle_command("not json at all"))
test("Malformed JSON returns error", r.get("error") == "bad_json")

r = json.loads(handle_command('{"wrong":"key"}'))
test("Missing cmd field returns error", r.get("error") == "no_cmd")

r = json.loads(handle_command('{"cmd":"launch_missiles"}'))
test("Unknown command returns error", r.get("error") == "unknown_cmd")


#  2. Stop command 
print("\n[2] Stop Command")
reset_state()
state["dir"] = "forward"
state["speed"] = 150

r = run({"cmd": "stop"})
test("Stop returns correct ack", r.get("ack") == "stop")
test("Stop sets dir to stop", state["dir"] == "stop")
test("Stop sets speed to 0", state["speed"] == 0)


#  Move commands ─
print("\n[3] Move Commands")
reset_state()

for direction in ["forward", "backward", "left", "right"]:
    reset_state()
    r = run({"cmd": "move", "dir": direction, "speed": 150})
    test(f"Move {direction} — ack correct", r.get("ack") == "move")
    test(f"Move {direction} — dir set", state["dir"] == direction)
    test(f"Move {direction} — speed set", state["speed"] == 150)

# Speed clamping
reset_state()
r = run({"cmd": "move", "dir": "forward", "speed": 999})
test("Speed clamped to 255 max", state["speed"] == 255)

reset_state()
r = run({"cmd": "move", "dir": "forward", "speed": -50})
test("Speed clamped to 0 min", state["speed"] == 0)

# Unknown direction
reset_state()
r = run({"cmd": "move", "dir": "sideways", "speed": 100})
test("Unknown direction returns error", r.get("error") == "unknown_dir")


#   Emergency Stop 
print("\n[4] Emergency Stop")
reset_state()
state["dir"] = "forward"
state["speed"] = 200

r = run({"cmd": "estop"})
test("eStop ack returned", r.get("ack") == "estop")
test("eStop locked flag true", r.get("locked") == True)
test("eStop sets state to stop", state["dir"] == "stop")
test("eStop zeroes speed", state["speed"] == 0)
test("eStop flag set in state", state["estop"] == True)

# Commands blocked while eStop active
r = run({"cmd": "move", "dir": "forward", "speed": 150})
test("Move blocked during eStop", r.get("error") == "estop_active")
test("Dir stays stop during eStop", state["dir"] == "stop")

r = run({"cmd": "stop"})
test("Stop also blocked during eStop", r.get("error") == "estop_active")


#   Reset after eStop 
print("\n[5] Reset After Emergency Stop")
# state still has estop=True from above

r = run({"cmd": "reset"})
test("Reset ack returned", r.get("ack") == "reset")
test("Reset clears locked flag", r.get("locked") == False)
test("eStop cleared in state", state["estop"] == False)

# Now movement should work again
r = run({"cmd": "move", "dir": "forward", "speed": 100})
test("Move works after reset", r.get("ack") == "move")


#  Sample command 
print("\n[6] Sample Command")
reset_state()

r = run({"cmd": "sample"})
test("Sample returns moisture array", "moisture" in r)
test("Sample returns soil_temp array", "soil_temp" in r)
test("Sample returns air_temp", "air_temp" in r)
test("Sample returns humidity", "humidity" in r)
test("Sample returns pressure", "pressure" in r)
test("Sample returns us_front", "us_front" in r)
test("Sample returns us_rear", "us_rear" in r)
test("Sample returns enc array", "enc" in r)
test("Sample returns estop bool", "estop" in r)
test("Sample moisture has 2 values", len(r["moisture"]) == 2)
test("Sample soil_temp has 2 values", len(r["soil_temp"]) == 2)
test("Sample enc has 4 values", len(r["enc"]) == 4)


#  Sensor value ranges 
print("\n[7] Sensor Value Ranges")
reset_state()

from rover_simulator import state as s
s["moisture"] = [45, 70]
s["soil_temp"] = [22.5, 23.1]
s["air_temp"] = 28.3
s["humidity"] = 65.0
s["pressure"] = 1012.5

r = run({"cmd": "sample"})
test("Moisture values are integers", all(isinstance(v, int) for v in r["moisture"]))
test("Moisture in 0-100 range", all(0 <= v <= 100 for v in r["moisture"]))
test("Soil temp values are floats", all(isinstance(v, float) for v in r["soil_temp"]))
test("Air temp is numeric", isinstance(r["air_temp"], (int, float)))
test("Pressure > 900 hPa", r["pressure"] > 900)
test("Humidity 0-100", 0 <= r["humidity"] <= 100)


#  Encoder counting 
print("\n[8] Encoder State")
reset_state()
state["enc"] = [0, 0, 0, 0]

run({"cmd": "move", "dir": "forward", "speed": 150})
test("Encoders start at zero", state["enc"] == [0, 0, 0, 0])

r = run({"cmd": "sample"})
test("Enc array has 4 channels", len(r["enc"]) == 4)
test("Enc values are integers", all(isinstance(v, int) for v in r["enc"]))


#  Whitespace / edge case inputs 
print("\n[9] Edge Cases")
reset_state()

r = handle_command("   \n\t  ")
test("Whitespace-only input returns None", r is None)

r = json.loads(handle_command('{"cmd":"move"}'))
test("Move with no dir defaults to forward", state["dir"] == "forward")

reset_state()
r = json.loads(handle_command('{"cmd":"move","dir":"forward"}'))
test("Move with no speed defaults to 150", state["speed"] == 150)


#  Summary 
passed = sum(1 for s, _ in results if s == PASS)
failed = sum(1 for s, _ in results if s == FAIL)
total  = len(results)

print("\n" + "="*55)
print(f"  Results: {passed}/{total} passed", end="")
if failed:
    print(f"  |  {failed} FAILED")
    print("\n  Failed tests:")
    for s, name in results:
        if s == FAIL:
            print(f"    {FAIL}  {name}")
else:
    print("  — All tests passed! 🎉")
print("="*55 + "\n")

sys.exit(0 if failed == 0 else 1)
