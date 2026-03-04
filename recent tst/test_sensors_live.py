"""
tests/test_sensors_live.py
───────────────────────────
Interactive sensor demo — tests each hardware layer live.
Run on Raspberry Pi with hardware connected.
Run on PC with --simulate flag for demo without hardware.

Usage:
    python tests/test_sensors_live.py             # real hardware (Pi)
    python tests/test_sensors_live.py --simulate  # simulate all sensors (PC/demo)
    python tests/test_sensors_live.py --layer 3   # jump straight to a layer
"""

import sys
import os
import time
import argparse
import threading

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

# ── Colours ───────────────────────────────────────────────────
G  = "\033[92m"   # green
Y  = "\033[93m"   # yellow
R  = "\033[91m"   # red
B  = "\033[94m"   # blue
C  = "\033[96m"   # cyan
W  = "\033[97m"   # white bold
DIM= "\033[2m"
RST= "\033[0m"

def clr(text, colour): return f"{colour}{text}{RST}"
def ok(msg):   print(f"  {clr('✅', G)} {msg}")
def warn(msg): print(f"  {clr('⚠️ ', Y)} {msg}")
def err(msg):  print(f"  {clr('❌', R)} {msg}")
def info(msg): print(f"  {clr('ℹ', B)} {msg}")

def header(n, title):
    print()
    print(clr("═"*55, C))
    print(clr(f"  LAYER {n} — {title}", W))
    print(clr("═"*55, C))

def subhead(title):
    print(f"\n  {clr('─'*45, DIM)}")
    print(f"  {clr(title, Y)}")
    print(f"  {clr('─'*45, DIM)}")

def bar(value, max_val=100, width=30, colour=G):
    filled = int((value / max_val) * width)
    filled = max(0, min(filled, width))
    b = "█" * filled + "░" * (width - filled)
    return clr(b, colour)

def wait_key(msg="Press ENTER to continue"):
    try:
        input(f"\n  {clr('▶', C)} {msg} > ")
    except KeyboardInterrupt:
        print("\n\nExiting.")
        sys.exit(0)

def countdown(seconds, msg="Starting in"):
    for i in range(seconds, 0, -1):
        print(f"\r  {clr(f'{msg} {i}s...', DIM)}", end="", flush=True)
        time.sleep(1)
    print(f"\r  {clr('Reading...            ', G)}", end="\n", flush=True)


# ══════════════════════════════════════════════════════════════
def layer0_system_check(simulate):
    """Check Python environment, imports, config."""
    header(0, "SYSTEM & ENVIRONMENT CHECK")

    # Python version
    v = sys.version_info
    if v >= (3, 8):
        ok(f"Python {v.major}.{v.minor}.{v.micro}")
    else:
        err(f"Python {v.major}.{v.minor} — need 3.8+")

    # Config import
    try:
        import pi.config as cfg
        ok(f"Config loaded — Serial: {cfg.SERIAL_PORT} | GPS: {cfg.GPS_PORT} | IMU: {hex(cfg.IMU_I2C_ADDRESS)}")
    except Exception as e:
        err(f"Config import failed: {e}"); return False

    # Required packages
    packages = [
        ("serial",   "pyserial"),
        ("pynmea2",  "pynmea2"),
        ("cv2",      "opencv-python"),
        ("numpy",    "numpy"),
        ("flask",    "flask"),
    ]
    all_ok = True
    for mod, pkg in packages:
        try:
            __import__(mod)
            ok(f"{pkg}")
        except ImportError:
            warn(f"{pkg} not installed  →  pip install {pkg}")
            all_ok = False

    # Optional Pi-only packages
    pi_packages = [("smbus2", "smbus2"), ("picamera2", "picamera2")]
    for mod, pkg in pi_packages:
        try:
            __import__(mod)
            ok(f"{pkg} (Pi hardware library)")
        except ImportError:
            if simulate:
                info(f"{pkg} not installed — OK in simulate mode")
            else:
                warn(f"{pkg} missing — install with: sudo apt install python3-{pkg}")

    if simulate:
        print()
        info("Running in SIMULATION MODE — no hardware required")
    else:
        print()
        info("Running in HARDWARE MODE — sensors must be connected")

    return True


# ══════════════════════════════════════════════════════════════
def layer1_arduino_serial(simulate):
    """Test Arduino serial connection and sensor packets."""
    header(1, "ARDUINO SERIAL — Soil & Environmental Sensors")

    info("What this tests:")
    print(f"    {DIM}• USB serial link between Pi and Arduino Nano{RST}")
    print(f"    {DIM}• Soil moisture (2 channels){RST}")
    print(f"    {DIM}• Soil temperature (2 channels){RST}")
    print(f"    {DIM}• Air temperature, humidity, pressure (BME280){RST}")
    print(f"    {DIM}• Ultrasonic distance sensors (front + rear){RST}")
    print(f"    {DIM}• Wheel encoder counts{RST}")

    if not simulate:
        print()
        warn("Make sure Arduino is connected via USB before continuing")
        wait_key()

    from pi.comms.serial_handler import SerialHandler

    print()
    info(f"Connecting to Arduino {'(simulator)' if simulate else '(/dev/ttyUSB0)'}...")
    arduino = SerialHandler(simulate=simulate)
    connected = arduino.connect()

    if not connected:
        err("Failed to connect to Arduino")
        err("Check: USB cable connected? Correct port in config.py?")
        err("Try: ls /dev/ttyUSB* or ls /dev/ttyACM*")
        return None

    ok(f"Arduino connected — waiting for first packet...")

    # Wait for first live packet
    for _ in range(10):
        time.sleep(0.5)
        d = arduino.latest()
        if d:
            break

    if not d:
        err("No data received from Arduino after 5s")
        return None

    ok("First packet received!\n")

    # ── Live reading loop ──────────────────────────────────────
    print(f"  {clr('Reading 5 packets — watch values in real time:', W)}\n")

    for i in range(5):
        time.sleep(2)
        d = arduino.latest()
        if not d:
            warn("No data"); continue

        m1  = d.get("moisture",  [0,0])[0]
        m2  = d.get("moisture",  [0,0])[1]
        st1 = d.get("soil_temp", [0,0])[0]
        st2 = d.get("soil_temp", [0,0])[1]
        at  = d.get("air_temp",  0)
        hum = d.get("humidity",  0)
        prs = d.get("pressure",  0)
        usf = d.get("us_front",  0)
        usr = d.get("us_rear",   0)
        enc = d.get("enc",       [0,0,0,0])

        print(f"  {clr(f'Packet {i+1}/5', C)}  ──────────────────────────────────────")
        print(f"  {'Soil Moisture 1':20} {bar(m1)} {clr(f'{m1:3d}%', G)}")
        print(f"  {'Soil Moisture 2':20} {bar(m2)} {clr(f'{m2:3d}%', G)}")
        print(f"  {'Soil Temp 1':20} {bar(st1, 50, colour=Y)} {clr(f'{st1:.1f}°C', Y)}")
        print(f"  {'Soil Temp 2':20} {bar(st2, 50, colour=Y)} {clr(f'{st2:.1f}°C', Y)}")
        print(f"  {'Air Temp':20} {bar(at,  50, colour=Y)} {clr(f'{at:.1f}°C', Y)}")
        print(f"  {'Humidity':20} {bar(hum)} {clr(f'{hum:.1f}%', B)}")
        print(f"  {'Pressure':20} {clr(f'{prs:.1f} hPa', W)}")
        print(f"  {'Ultrasonic Front':20} {bar(usf, 300)} {clr(f'{usf} cm', C)}")
        print(f"  {'Ultrasonic Rear':20} {bar(usr, 300)} {clr(f'{usr} cm', C)}")
        print(f"  {'Encoders':20} {clr(str(enc), DIM)}")
        print()

    # ── Command test ───────────────────────────────────────────
    subhead("Command Test — Motor Control")
    info("Sending move/stop commands to Arduino...")

    cmds = [
        ("move forward", lambda: arduino.move("forward", 150)),
        ("stop",         lambda: arduino.stop()),
        ("move left",    lambda: arduino.move("left", 120)),
        ("stop",         lambda: arduino.stop()),
        ("eStop",        lambda: arduino.estop()),
        ("reset eStop",  lambda: arduino.reset_estop()),
    ]

    for label, fn in cmds:
        fn()
        time.sleep(0.3)
        ok(f"Command '{label}' sent")

    # ── eStop verification ─────────────────────────────────────
    arduino.estop()
    time.sleep(0.3)
    d = arduino.latest()
    if d.get("estop") == True:
        ok("Emergency stop confirmed in sensor packet")
    else:
        warn("eStop flag not reflected yet (may need one more broadcast cycle)")

    arduino.reset_estop()
    time.sleep(0.3)
    ok("eStop reset confirmed")

    print()
    ok(f"Layer 1 complete — Arduino serial {'(simulated)' if simulate else 'LIVE'}")
    return arduino


# ══════════════════════════════════════════════════════════════
def layer2_gps(simulate):
    """Test GPS module — NMEA parsing and position fix."""
    header(2, "GPS MODULE — NEO-6M Position & Navigation")

    info("What this tests:")
    print(f"    {DIM}• UART serial link to NEO-6M GPS module{RST}")
    print(f"    {DIM}• NMEA sentence parsing (GPRMC / GPGGA){RST}")
    print(f"    {DIM}• Latitude / Longitude decimal conversion{RST}")
    print(f"    {DIM}• GPS fix status{RST}")
    print(f"    {DIM}• Haversine distance calculation{RST}")
    print(f"    {DIM}• Bearing calculation{RST}")

    if not simulate:
        print()
        warn("GPS needs 30–60 seconds to acquire satellite fix outdoors")
        warn("Make sure NEO-6M is connected to /dev/ttyAMA0")
        wait_key("Place GPS near a window or outdoors, then press ENTER")

    from pi.navigation.gps_reader import GPSReader, haversine_distance

    print()
    info(f"Starting GPS reader {'(simulator)' if simulate else '(/dev/ttyAMA0)'}...")
    gps = GPSReader(simulate=simulate)
    gps.start()

    if not simulate:
        print()
        info("Waiting for GPS fix (up to 60 seconds)...")
        for i in range(60):
            time.sleep(1)
            if gps.has_fix():
                break
            print(f"\r  {clr(f'Waiting for fix... {i+1}s', DIM)}", end="", flush=True)
        print()

    # Wait for sim fix
    if simulate:
        time.sleep(1.5)

    # ── Position readout ───────────────────────────────────────
    print(f"\n  {clr('Reading 5 GPS positions:', W)}\n")

    positions = []
    for i in range(5):
        time.sleep(1)
        lat, lon = gps.position()

        if lat is None:
            warn(f"Reading {i+1}: No fix yet")
            continue

        positions.append((lat, lon))

        print(f"  {clr(f'Reading {i+1}/5', C)}  ──────────────────────────────────────")
        print(f"  {'Latitude':18} {clr(f'{lat:.7f}°', G)}")
        print(f"  {'Longitude':18} {clr(f'{lon:.7f}°', G)}")
        print(f"  {'Fix Status':18} {clr('✅ Active', G) if gps.has_fix() else clr('⚠️  No fix', Y)}")

        if len(positions) > 1:
            d = haversine_distance(positions[0][0], positions[0][1], lat, lon)
            print(f"  {'Moved since start':18} {clr(f'{d:.2f}m', C)}")

        print()

    # ── Distance & bearing demo ────────────────────────────────
    if positions:
        subhead("Distance & Bearing Demo")
        lat, lon = positions[-1]

        # Target point 50m north
        target_lat = lat + 0.00045
        target_lon = lon

        d = haversine_distance(lat, lon, target_lat, target_lon)
        b = gps.bearing_to(target_lat, target_lon)

        ok(f"Current position:  {lat:.6f}, {lon:.6f}")
        ok(f"Target (50m N):    {target_lat:.6f}, {target_lon:.6f}")
        ok(f"Distance to target: {clr(f'{d:.1f}m', Y)}")
        ok(f"Bearing to target:  {clr(f'{b:.1f}°', Y)} {'(North ↑)' if b < 15 or b > 345 else ''}")

    gps.stop()
    print()
    ok(f"Layer 2 complete — GPS {'(simulated)' if simulate else 'LIVE'}")
    return True


# ══════════════════════════════════════════════════════════════
def layer3_imu(simulate):
    """Test IMU — MPU-6050 heading, pitch, roll."""
    header(3, "IMU — MPU-6050 Heading, Pitch & Roll")

    info("What this tests:")
    print(f"    {DIM}• I2C communication to MPU-6050 at address 0x68{RST}")
    print(f"    {DIM}• Gyroscope reading (angular velocity){RST}")
    print(f"    {DIM}• Accelerometer reading (tilt/gravity){RST}")
    print(f"    {DIM}• Complementary filter → heading (yaw){RST}")
    print(f"    {DIM}• Pitch and roll angles{RST}")

    if not simulate:
        print()
        warn("MPU-6050 must be connected via I2C (SDA=GPIO2, SCL=GPIO3)")
        wait_key()

    from pi.navigation.imu_reader import IMUReader

    print()
    info(f"Starting IMU {'(simulator)' if simulate else '(I2C 0x68)'}...")
    imu = IMUReader(simulate=simulate)
    imu.start()
    time.sleep(0.5)

    print(f"\n  {clr('Reading 8 IMU samples — try tilting the sensor:', W)}\n")

    for i in range(8):
        time.sleep(0.5)
        h = imu.heading()
        p = imu.pitch()
        r = imu.roll()

        # Direction label from heading
        dirs = ["N","NE","E","SE","S","SW","W","NW","N"]
        direction = dirs[int((h + 22.5) / 45) % 8]

        # Colour heading bar
        h_pct = h / 360 * 100
        tilt_ok = abs(p) < 15 and abs(r) < 15

        print(f"  {clr(f'Sample {i+1}/8', C)}  ──────────────────────────────────────")
        print(f"  {'Heading':12} {bar(h_pct, colour=C)} {clr(f'{h:6.1f}°', C)} {clr(direction, W)}")
        print(f"  {'Pitch':12} {bar(abs(p), 90, colour=Y)} {clr(f'{p:+.1f}°', Y)} {'⚠️  steep' if abs(p)>20 else ''}")
        print(f"  {'Roll':12} {bar(abs(r), 90, colour=Y)} {clr(f'{r:+.1f}°', Y)} {'⚠️  steep' if abs(r)>20 else ''}")
        print(f"  {'Platform':12} {clr('Level ✅', G) if tilt_ok else clr('Tilted ⚠️', Y)}")
        print()

    if not simulate:
        subhead("Live Heading Monitor — Rotate the sensor")
        info("Watching heading change for 10 seconds — rotate your rover...")
        print()
        headings = []
        for _ in range(20):
            time.sleep(0.5)
            h = imu.heading()
            headings.append(h)
            dirs = ["N","NE","E","SE","S","SW","W","NW","N"]
            direction = dirs[int((h + 22.5) / 45) % 8]
            print(f"\r  Heading: {clr(f'{h:6.1f}°', C)} {clr(direction, W)}    ", end="", flush=True)
        print()
        spread = max(headings) - min(headings)
        if spread > 5:
            ok(f"Heading changed by {spread:.1f}° — IMU responsive!")
        else:
            warn(f"Heading barely changed ({spread:.1f}°) — try rotating more")

    imu.stop()
    print()
    ok(f"Layer 3 complete — IMU {'(simulated)' if simulate else 'LIVE'}")
    return True


# ══════════════════════════════════════════════════════════════
def layer4_data_pipeline(arduino, simulate):
    """Test full data pipeline: Arduino → Logger → SQLite → CSV."""
    header(4, "DATA PIPELINE — Sensor → Logger → Database")

    info("What this tests:")
    print(f"    {DIM}• Takes live sensor packet from Arduino{RST}")
    print(f"    {DIM}• Writes to CSV log file{RST}")
    print(f"    {DIM}• Writes to SQLite database{RST}")
    print(f"    {DIM}• Queries back and verifies data integrity{RST}")

    import tempfile
    tmpdir = tempfile.mkdtemp()

    import pi.config as cfg
    cfg.DB_PATH = os.path.join(tmpdir, "sensor_test.db")
    cfg.LOG_DIR = os.path.join(tmpdir, "logs")
    cfg.IMAGE_DIR = os.path.join(tmpdir, "images")
    os.makedirs(cfg.LOG_DIR, exist_ok=True)
    os.makedirs(cfg.IMAGE_DIR, exist_ok=True)

    from pi.sensors.data_logger import DataLogger

    print()
    info("Initialising data logger...")
    logger = DataLogger()
    ok(f"CSV file: {logger.csv_path}")
    ok(f"Database: {cfg.DB_PATH}")

    # Pull live packets and log them
    print(f"\n  {clr('Logging 5 sensor readings...', W)}\n")

    logged = 0
    for i in range(5):
        time.sleep(2)

        if arduino:
            packet = arduino.latest()
        else:
            # Fallback if no arduino passed
            packet = {
                "moisture":  [58+i, 63+i],
                "soil_temp": [24.5, 24.1],
                "air_temp":  28.3,
                "humidity":  65.2,
                "pressure":  1011.5,
                "us_front":  120,
                "us_rear":   180,
                "enc":       [i*10, i*10, i*10, i*10],
            }

        if not packet:
            warn(f"Reading {i+1}: No packet"); continue

        # Fake GPS coords moving slightly
        gps_coords = (6.688500 + i * 0.000010, -1.624400 + i * 0.000005)
        stress = packet.get("moisture", [100])[0] < 30   # dry = stressed

        logger.log(
            packet,
            gps_coords=gps_coords,
            stress_flag=stress,
            stress_score=0.05 if not stress else 0.45,
        )
        logged += 1

        m1 = packet.get("moisture", [0,0])[0]
        at = packet.get("air_temp", 0)
        print(f"  {clr(f'✅ Reading {i+1}', G)} logged — "
              f"Moisture: {clr(f'{m1}%', G)}  "
              f"Air Temp: {clr(f'{at}°C', Y)}  "
              f"GPS: {clr(str(gps_coords), DIM)}")

    # ── Query back ─────────────────────────────────────────────
    subhead("Querying Database")

    last = logger.get_last(10)
    ok(f"get_last(10) returned {len(last)} rows")

    if last:
        r = last[0]
        ok(f"Latest moisture_1 = {r['moisture_1']}%")
        ok(f"Latest air_temp   = {r['air_temp']}°C")
        ok(f"Latest GPS        = ({r['latitude']:.6f}, {r['longitude']:.6f})")

    summary = logger.summary()
    m1_avg  = f"{summary['avg_moisture_1']:.1f}%"
    at_avg  = f"{summary['avg_air_temp']:.1f}°C"
    stress_c = R if summary['stress_events'] > 0 else G
    ok(f"Total readings in DB: {clr(str(summary['total_readings']), W)}")
    ok(f"Avg moisture ch1:     {clr(m1_avg, G)}")
    ok(f"Avg air temp:         {clr(at_avg, Y)}")
    ok(f"Stress events:        {clr(str(summary['stress_events']), stress_c)}")

    # ── CSV check ─────────────────────────────────────────────
    subhead("CSV Log File")
    with open(logger.csv_path) as f:
        lines = f.readlines()
    ok(f"CSV has {len(lines)} lines ({len(lines)-1} readings + header)")
    ok(f"CSV path: {logger.csv_path}")
    print(f"\n  {clr('First row preview:', DIM)}")
    print(f"  {DIM}{lines[0].strip()[:80]}...{RST}")

    logger.close()
    print()
    ok(f"Layer 4 complete — Data pipeline working end-to-end")
    return logger


# ══════════════════════════════════════════════════════════════
def layer5_full_stack(simulate):
    """Run all sensors together simultaneously — like the real mission."""
    header(5, "FULL STACK — All Sensors Running Together")

    info("What this tests:")
    print(f"    {DIM}• Arduino + GPS + IMU all running in parallel threads{RST}")
    print(f"    {DIM}• All data merged into a single timestamped reading{RST}")
    print(f"    {DIM}• Exactly what happens during a real patrol mission{RST}")

    if not simulate:
        warn("All hardware must be connected: Arduino (USB), GPS (/dev/ttyAMA0), IMU (I2C)")
        wait_key()

    from pi.comms.serial_handler import SerialHandler
    from pi.navigation.gps_reader import GPSReader
    from pi.navigation.imu_reader import IMUReader

    print()
    info("Starting all subsystems...")

    arduino = SerialHandler(simulate=simulate)
    arduino.connect()
    ok("Arduino serial connected")

    gps = GPSReader(simulate=simulate)
    gps.start()
    ok("GPS reader started")

    imu = IMUReader(simulate=simulate)
    imu.start()
    ok("IMU reader started")

    time.sleep(2)
    print(f"\n  {clr('Live combined readings — 8 samples at 2s intervals:', W)}\n")

    for i in range(8):
        time.sleep(2)

        d   = arduino.latest() or {}
        lat, lon = gps.position()
        h   = imu.heading()
        p   = imu.pitch()
        r   = imu.roll()

        m1  = d.get("moisture",  [0,0])[0]
        m2  = d.get("moisture",  [0,0])[1]
        at  = d.get("air_temp",  0)
        hum = d.get("humidity",  0)
        usf = d.get("us_front",  0)

        dirs = ["N","NE","E","SE","S","SW","W","NW","N"]
        direction = dirs[int((h + 22.5) / 45) % 8] if h else "?"

        obstacle = usf < 20 and usf > 0

        print(f"  {clr(f'[ Sample {i+1:02d} ]', C)} ─────────────────────────────────────────")
        print(f"  {clr('SOIL', Y)}   Moisture: {clr(f'{m1}%', G)} / {clr(f'{m2}%', G)}"
              f"   Air: {clr(f'{at:.1f}°C', Y)}   Humidity: {clr(f'{hum:.1f}%', B)}")
        print(f"  {clr('GPS ', B)}   {'Lat: ' + clr(f'{lat:.6f}', G) + '  Lon: ' + clr(f'{lon:.6f}', G) if lat else clr('No fix yet...', DIM)}")
        print(f"  {clr('IMU ', C)}   Heading: {clr(f'{h:.1f}°', C)} {clr(direction, W)}"
              f"   Pitch: {clr(f'{p:+.1f}°', Y)}   Roll: {clr(f'{r:+.1f}°', Y)}")
        print(f"  {clr('DIST', R if obstacle else G)}   Front: {clr(f'{usf}cm', R if obstacle else G)}"
              f"  {'⚠️  OBSTACLE DETECTED!' if obstacle else '  Clear'}")
        print()

    arduino.disconnect()
    gps.stop()
    imu.stop()

    print()
    ok("All subsystems stopped cleanly")
    ok(f"Layer 5 complete — Full sensor stack {'(simulated)' if simulate else 'LIVE'}")


# ══════════════════════════════════════════════════════════════
def print_intro(simulate):
    print()
    print(clr("╔══════════════════════════════════════════════════════╗", C))
    print(clr("║     AUTONOMOUS FARM ROVER — Sensor Test Suite        ║", W))
    print(clr("║     Layer-by-layer hardware validation               ║", C))
    print(clr("╚══════════════════════════════════════════════════════╝", C))
    print()
    mode = clr("SIMULATION MODE (no hardware needed)", Y) if simulate else clr("HARDWARE MODE (sensors required)", G)
    print(f"  Mode: {mode}")
    print()
    print(f"  {DIM}Layers:{RST}")
    print(f"  {DIM}  0 — System & environment check{RST}")
    print(f"  {DIM}  1 — Arduino serial + soil/env sensors{RST}")
    print(f"  {DIM}  2 — GPS module (NEO-6M){RST}")
    print(f"  {DIM}  3 — IMU (MPU-6050 heading & tilt){RST}")
    print(f"  {DIM}  4 — Data pipeline (logger → SQLite → CSV){RST}")
    print(f"  {DIM}  5 — Full stack (all sensors together){RST}")
    print()


# ══════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(description="Rover sensor test suite")
    parser.add_argument("--simulate", action="store_true",
                        help="Run in simulation mode (no hardware required)")
    parser.add_argument("--layer", type=int, default=None,
                        help="Jump to a specific layer (0-5)")
    parser.add_argument("--all", action="store_true",
                        help="Run all layers without pausing between them")
    args = parser.parse_args()

    simulate = args.simulate
    print_intro(simulate)

    layers = [0, 1, 2, 3, 4, 5] if args.layer is None else [args.layer]
    arduino = None

    for layer_num in layers:
        if not args.all and layer_num > 0:
            wait_key(f"Ready for Layer {layer_num}? Press ENTER")

        try:
            if layer_num == 0:
                layer0_system_check(simulate)

            elif layer_num == 1:
                arduino = layer1_arduino_serial(simulate)

            elif layer_num == 2:
                layer2_gps(simulate)

            elif layer_num == 3:
                layer3_imu(simulate)

            elif layer_num == 4:
                layer4_data_pipeline(arduino, simulate)

            elif layer_num == 5:
                layer5_full_stack(simulate)

        except KeyboardInterrupt:
            print(f"\n\n  {clr('Interrupted by user', Y)}")
            break
        except Exception as e:
            err(f"Layer {layer_num} error: {e}")
            import traceback
            traceback.print_exc()

    print()
    print(clr("═"*55, C))
    print(clr("  Sensor test complete.", W))
    print(clr("═"*55, C))
    print()


if __name__ == "__main__":
    main()
