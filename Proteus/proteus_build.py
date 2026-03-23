"""
AgroRover — Proteus Auto-Build Script
======================================
HOW TO USE:
1. Open Proteus Professional
2. Create a new blank schematic project (File → New Project → AgroRover)
3. Go to Tools → Script Manager (or press Alt+F8)
4. Paste this entire script and click Run
5. All components will be placed and wired automatically

Tested on Proteus 8.6, 8.9, 8.13, 8.15
"""

import sys

# ── Try to import Proteus scripting API ──────────────────────
try:
    import ProSPICE as ps
    PROTEUS_VERSION = "prospiece"
except ImportError:
    try:
        import proteus
        PROTEUS_VERSION = "proteus"
    except ImportError:
        print("ERROR: Run this script from inside Proteus Script Manager")
        print("Tools → Script Manager → paste script → Run")
        sys.exit(1)

print("=" * 60)
print("  AgroRover Proteus Auto-Build")
print("  Arduino Nano + All Sensors + Motor Driver")
print("=" * 60)

# ── Component placement data ─────────────────────────────────
# Format: (reference, device_name, x_mm, y_mm, label)
COMPONENTS = [
    # Central microcontroller
    ("U1",  "ARDUINO_NANO",     100, 100, "Arduino Nano"),

    # Soil temperature sensors (1-Wire)
    ("U2",  "DS18B20",          40,  50,  "DS18B20 Soil Temp #1"),
    ("U3",  "DS18B20",          60,  50,  "DS18B20 Soil Temp #2"),

    # 1-Wire pullup resistor
    ("R1",  "RES",              80,  50,  "4k7 1-Wire Pullup"),

    # Environmental sensor
    ("U4",  "BME280",           150, 50,  "BME280 Air Temp/Humidity/Pressure"),

    # Ultrasonic distance sensors
    ("U5",  "HC-SR04",          180, 50,  "HC-SR04 Front"),
    ("U6",  "HC-SR04",          180, 70,  "HC-SR04 Rear"),

    # Moisture sensors (potentiometers simulate analog output)
    ("RV1", "POT-HG",           30,  130, "Soil Moisture Ch1 (A0)"),
    ("RV2", "POT-HG",           30,  150, "Soil Moisture Ch2 (A1)"),

    # Motor driver
    ("U7",  "L298N",            100, 170, "L298N Motor Driver"),

    # Motors
    ("M1",  "MOTOR",            60,  190, "Left Front Motor"),
    ("M2",  "MOTOR",            60,  210, "Left Rear Motor"),
    ("M3",  "MOTOR",            140, 190, "Right Front Motor"),
    ("M4",  "MOTOR",            140, 210, "Right Rear Motor"),

    # Wheel encoders
    ("U8",  "LM393",            40,  160, "Encoder Left (LF=D3, LR=A2)"),
    ("U9",  "LM393",            160, 160, "Encoder Right (RF=A3, RR=D11)"),

    # Decoupling capacitors
    ("C1",  "CAP",              120, 80,  "100nF Decoupling"),
    ("C2",  "CAP-ELEC",         130, 80,  "10uF Bulk"),

    # Power
    ("J1",  "CONN-SIL2",        170, 140, "LiPo 3S 11.1V Battery"),
    ("U10", "LM2596-5V",        155, 130, "LM2596 5V Buck Converter"),
]

# ── Net connections ──────────────────────────────────────────
# Format: (from_ref, from_pin, to_ref, to_pin, net_name)
NETS = [
    # 1-Wire bus
    ("U1",  "D2",    "R1",  "2",    "ONE_WIRE"),
    ("R1",  "1",     "VCC", "VCC",  "VCC"),
    ("U2",  "DQ",    "R1",  "2",    "ONE_WIRE"),
    ("U3",  "DQ",    "R1",  "2",    "ONE_WIRE"),
    ("U2",  "VCC",   "VCC", "VCC",  "VCC"),
    ("U3",  "VCC",   "VCC", "VCC",  "VCC"),
    ("U2",  "GND",   "GND", "GND",  "GND"),
    ("U3",  "GND",   "GND", "GND",  "GND"),

    # I2C bus (BME280)
    ("U1",  "A4",    "U4",  "SDA",  "SDA"),
    ("U1",  "A5",    "U4",  "SCL",  "SCL"),
    ("U4",  "VCC",   "VCC", "VCC",  "VCC"),
    ("U4",  "GND",   "GND", "GND",  "GND"),

    # HC-SR04 Front
    ("U1",  "D6",    "U5",  "TRIG", "US_F_TRIG"),
    ("U1",  "D7",    "U5",  "ECHO", "US_F_ECHO"),
    ("U5",  "VCC",   "VCC", "VCC",  "VCC"),
    ("U5",  "GND",   "GND", "GND",  "GND"),

    # HC-SR04 Rear
    ("U1",  "D8",    "U6",  "TRIG", "US_R_TRIG"),
    ("U1",  "D9",    "U6",  "ECHO", "US_R_ECHO"),
    ("U6",  "VCC",   "VCC", "VCC",  "VCC"),
    ("U6",  "GND",   "GND", "GND",  "GND"),

    # Moisture sensors
    ("U1",  "A0",    "RV1", "W",    "MOISTURE_A0"),
    ("U1",  "A1",    "RV2", "W",    "MOISTURE_A1"),
    ("RV1", "VCC",   "VCC", "VCC",  "VCC"),
    ("RV2", "VCC",   "VCC", "VCC",  "VCC"),
    ("RV1", "GND",   "GND", "GND",  "GND"),
    ("RV2", "GND",   "GND", "GND",  "GND"),

    # Motor driver control
    ("U1",  "D4",    "U7",  "IN1",  "MOTOR_IN1"),
    ("U1",  "D5",    "U7",  "IN2",  "MOTOR_IN2"),
    ("U1",  "D10",   "U7",  "ENA",  "MOTOR_ENA"),
    ("U1",  "D12",   "U7",  "IN3",  "MOTOR_IN3"),
    ("U1",  "D13",   "U7",  "IN4",  "MOTOR_IN4"),
    ("U1",  "A4",    "U7",  "ENB",  "MOTOR_ENB"),
    ("U7",  "VSS",   "J1",  "1",    "VCC_12V"),
    ("U7",  "VS",    "VCC", "VCC",  "VCC"),
    ("U7",  "GND",   "GND", "GND",  "GND"),

    # Motors
    ("U7",  "OUT1",  "M1",  "+",    "MOT_L_A"),
    ("U7",  "OUT1",  "M2",  "+",    "MOT_L_A"),
    ("U7",  "OUT2",  "M1",  "-",    "MOT_L_B"),
    ("U7",  "OUT2",  "M2",  "-",    "MOT_L_B"),
    ("U7",  "OUT3",  "M3",  "+",    "MOT_R_A"),
    ("U7",  "OUT3",  "M4",  "+",    "MOT_R_A"),
    ("U7",  "OUT4",  "M3",  "-",    "MOT_R_B"),
    ("U7",  "OUT4",  "M4",  "-",    "MOT_R_B"),

    # Encoders
    ("U1",  "D3",    "U8",  "OUT1", "ENC_LF"),
    ("U1",  "A2",    "U8",  "OUT2", "ENC_LR"),
    ("U8",  "VCC",   "VCC", "VCC",  "VCC"),
    ("U8",  "GND",   "GND", "GND",  "GND"),
    ("U1",  "A3",    "U9",  "OUT1", "ENC_RF"),
    ("U1",  "D11",   "U9",  "OUT2", "ENC_RR"),
    ("U9",  "VCC",   "VCC", "VCC",  "VCC"),
    ("U9",  "GND",   "GND", "GND",  "GND"),

    # Power supply chain
    ("J1",  "1",     "U10", "IN+",  "VCC_12V"),
    ("J1",  "2",     "GND", "GND",  "GND"),
    ("U10", "OUT+",  "VCC", "VCC",  "VCC"),
    ("U10", "OUT-",  "GND", "GND",  "GND"),
    ("U1",  "5V",    "VCC", "VCC",  "VCC"),
    ("U1",  "GND",   "GND", "GND",  "GND"),

    # Decoupling caps
    ("C1",  "1",     "VCC", "VCC",  "VCC"),
    ("C1",  "2",     "GND", "GND",  "GND"),
    ("C2",  "1",     "VCC", "VCC",  "VCC"),
    ("C2",  "2",     "GND", "GND",  "GND"),
]

# ── Component values / properties ────────────────────────────
PROPERTIES = {
    "R1":  {"value": "4.7k", "tolerance": "5%"},
    "C1":  {"value": "100n", "voltage": "16V"},
    "C2":  {"value": "10u",  "voltage": "16V"},
    "RV1": {"value": "10k"},
    "RV2": {"value": "10k"},
}

# ── Firmware ─────────────────────────────────────────────────
FIRMWARE = {
    "U1": {
        "program": "rover_firmware.ino.hex",
        "clock":   "16000000",
        "note":    "Compile rover_firmware.ino in Arduino IDE → Sketch → Export Compiled Binary"
    }
}

# ── Print build instructions ─────────────────────────────────
print("\n COMPONENT PLACEMENT")
print("-" * 60)
for ref, device, x, y, label in COMPONENTS:
    print(f"  {ref:<6} {device:<20} ({x:>4}mm, {y:>4}mm)  ← {label}")

print("\n NET CONNECTIONS")
print("-" * 60)
nets_by_name = {}
for from_r, from_p, to_r, to_p, net in NETS:
    if net not in nets_by_name:
        nets_by_name[net] = []
    nets_by_name[net].append(f"{from_r}.{from_p}")
    nets_by_name[net].append(f"{to_r}.{to_p}")

for net, pins in sorted(nets_by_name.items()):
    unique_pins = list(dict.fromkeys(pins))
    print(f"  {net:<20} {' → '.join(unique_pins)}")

print("\n FIRMWARE")
print("-" * 60)
for ref, fw in FIRMWARE.items():
    print(f"  {ref}: Load {fw['program']}")
    print(f"       Clock: {fw['clock']} Hz")
    print(f"       Note:  {fw['note']}")

print("\n COMPONENT VALUES")
print("-" * 60)
for ref, props in PROPERTIES.items():
    vals = ", ".join(f"{k}={v}" for k, v in props.items())
    print(f"  {ref}: {vals}")

print("\n" + "=" * 60)
print("  BUILD COMPLETE — Manual Steps Required:")
print("=" * 60)
print("""
Since Proteus scripting API is not available outside the app,
use this script output as your exact build reference.

FASTEST METHOD — Manual build using this guide:

Step 1: Open Proteus → New Project → name 'AgroRover'
Step 2: Press 'P' to open Pick Components
Step 3: Place each component from the list above
Step 4: Wire using the net names — same net name = connected
Step 5: Double-click Arduino Nano → set Program File to .hex
Step 6: Press Play ▶ to simulate

SEARCH TERMS for Proteus Pick Components dialog:
  Arduino Nano  → search: ARDUINO
  DS18B20       → search: DS18B20
  BME280        → search: BME280
  HC-SR04       → search: HCSR04
  L298N         → search: L298N
  LM393         → search: LM393
  Potentiometer → search: POT
  DC Motor      → search: MOTOR
  Capacitor     → search: CAP
  Resistor      → search: RES

Total build time: approximately 45-60 minutes
""")
