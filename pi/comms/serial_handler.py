"""
comms/serial_handler.py
─
Manages the USB serial link between Raspberry Pi and Arduino.
Runs a background reader thread so the main mission loop
never blocks waiting for data.

Usage:
    from pi.comms.serial_handler import SerialHandler

    arduino = SerialHandler()
    arduino.connect()

    # Send a command
    arduino.send({"cmd": "move", "dir": "forward", "speed": 150})

    # Get latest sensor reading
    data = arduino.latest()

    arduino.disconnect()
"""

import json
import os
import threading
import time
import logging
from collections import deque
from typing import Optional, Dict, Any

log = logging.getLogger(__name__)


class SerialHandler:
    """
    Thread-safe serial interface to the Arduino.
    Uses a background thread to continuously read incoming JSON
    so the mission loop can pull the latest data at any time.
    """

    def __init__(self, port: str = None, baud: int = None, simulate: bool = False):
        from pi.config import SERIAL_PORT, SERIAL_BAUD
        self.port     = port or SERIAL_PORT
        self.baud     = baud or SERIAL_BAUD
        self.simulate = simulate        # True = use simulator, no real hardware

        self._serial   = None
        self._lock     = threading.Lock()
        self._running  = False
        self._thread   = None
        self._latest: Dict[str, Any] = {}
        self._history  = deque(maxlen=100)   # last 100 packets
        self._ready    = False
        self._callbacks = []             # optional listeners

    #  Connection 

    def connect(self) -> bool:
        if self.simulate:
            log.info("SerialHandler: simulation mode — no real serial port")
            self._running = True
            self._thread  = threading.Thread(target=self._simulate_loop, daemon=True)
            self._thread.start()
            self._ready = True
            return True

        try:
            import serial
            port = self._resolve_port()
            if not port:
                log.error("SerialHandler: no Arduino found on any known port")
                return False
            self._serial = serial.Serial(port, self.baud, timeout=2)
            self.port = port
            time.sleep(2)   # Arduino resets on serial connect — wait for it
            self._running = True
            self._thread  = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            log.info(f"SerialHandler: connected to {self.port} @ {self.baud}")
            return True
        except Exception as e:
            log.error(f"SerialHandler: connect failed — {e}")
            return False

    def _resolve_port(self) -> Optional[str]:
        """
        Try the configured port first, then fall back to scanning common
        Nano ports — CH340 clones enumerate as ttyUSB0, genuine/bootloader
        boards often show up as ttyACM0, and either can shift to *1 if
        something else was plugged in first.
        """
        import glob
        candidates = [self.port] + sorted(
            glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")
        )
        seen = set()
        for p in candidates:
            if p in seen or not p:
                continue
            seen.add(p)
            if os.path.exists(p):
                if p != self.port:
                    log.warning(f"SerialHandler: {self.port} not found — using {p} instead")
                return p
        return None

    def disconnect(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)
        if self._serial and self._serial.is_open:
            self._serial.close()
        log.info("SerialHandler: disconnected")

    def is_ready(self) -> bool:
        return self._ready

    #  Send 

    def send(self, command: Dict[str, Any]) -> bool:
        """Send a JSON command to the Arduino."""
        raw = json.dumps(command) + "\n"
        if self.simulate:
            # Feed into simulator's command handler
            from arduino.rover_simulator import handle_command
            response = handle_command(raw.strip())
            if response:
                self._ingest(response)
            return True
        if not self._serial or not self._serial.is_open:
            log.warning("SerialHandler: cannot send — port not open")
            return False
        try:
            with self._lock:
                self._serial.write(raw.encode("utf-8"))
                self._serial.flush()
            return True
        except Exception as e:
            log.error(f"SerialHandler: send error — {e}")
            return False

    #  Convenience command wrappers 

    def move(self, direction: str, speed: int = None):
        from pi.config import SPEED_CRUISE
        return self.send({"cmd": "move", "dir": direction, "speed": speed or SPEED_CRUISE})

    def stop(self):
        return self.send({"cmd": "stop"})

    def estop(self):
        return self.send({"cmd": "estop"})

    def reset_estop(self):
        return self.send({"cmd": "reset"})

    def request_sample(self):
        return self.send({"cmd": "sample"})

    #  Data access ─

    def latest(self) -> Dict[str, Any]:
        """Return the most recent full sensor packet."""
        with self._lock:
            return dict(self._latest)

    def history(self) -> list:
        """Return the last 100 sensor packets."""
        with self._lock:
            return list(self._history)

    def add_callback(self, fn):
        """Register a function to be called on every new packet."""
        self._callbacks.append(fn)

    #  Internal 

    def _read_loop(self):
        """Background thread — reads lines from Arduino serial."""
        while self._running:
            try:
                if self._serial and self._serial.in_waiting:
                    line = self._serial.readline().decode("utf-8", errors="ignore").strip()
                    if line:
                        self._ingest(line)
                else:
                    time.sleep(0.01)
            except Exception as e:
                log.error(f"SerialHandler: read error — {e}")
                time.sleep(0.5)

    def _simulate_loop(self):
        """Background thread — pulls from simulator's broadcast loop."""
        import sys, os
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "arduino"))
        from rover_simulator import state
        import random

        while self._running:
            # Build a realistic simulated packet
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
            self._ingest(json.dumps(packet))
            time.sleep(2.0)

    def _ingest(self, raw: str):
        """Parse a raw JSON line and update internal state."""
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            return

        # Mark ready once we see the firmware ready signal
        if data.get("status") == "ready":
            self._ready = True
            log.info(f"Arduino ready — fw {data.get('fw', '?')}")
            with self._lock:
                self._latest.update(data)
                self._latest["_ts"] = time.time()
                self._history.append(dict(self._latest))
            return

        # Store full sensor packets and stateful acknowledgements.
        if not any(k in data for k in ("moisture", "alert", "ack", "error", "status")):
            return

        if data.get("ack") == "estop" or data.get("locked") is True:
            data["estop"] = True
        elif data.get("ack") == "reset" or data.get("locked") is False:
            data["estop"] = False

        ts = time.time()
        data["_ts"] = ts

        with self._lock:
            self._latest.update(data)
            self._history.append(dict(self._latest))

        # Obstacle alert — log it
        if "alert" in data:
            log.warning(f"OBSTACLE ALERT: {data.get('dist_cm')} cm")

        # Fire callbacks
        for fn in self._callbacks:
            try:
                fn(data)
            except Exception as e:
                log.error(f"Callback error: {e}")
