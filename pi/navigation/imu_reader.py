"""
navigation/imu_reader.py
─────────────────────────
Reads heading and tilt from MPU-6050 over I2C.
Heading is computed from gyroscope integration (yaw).
For a proper magnetometer-fused heading, add an HMC5883L later.

Usage:
    from pi.navigation.imu_reader import IMUReader

    imu = IMUReader()
    imu.start()
    heading = imu.heading()   # 0–360 degrees
    imu.stop()
"""

import threading
import time
import math
import logging
from typing import Optional

log = logging.getLogger(__name__)

# MPU-6050 register addresses
PWR_MGMT_1   = 0x6B
GYRO_XOUT_H  = 0x43
ACCEL_XOUT_H = 0x3B
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C

# Scale factors
GYRO_SCALE   = 131.0   # LSB/(°/s) at ±250°/s
ACCEL_SCALE  = 16384.0 # LSB/g at ±2g


class IMUReader:

    def __init__(self, address: int = None, simulate: bool = False):
        from pi.config import IMU_I2C_ADDRESS
        self.address  = address or IMU_I2C_ADDRESS
        self.simulate = simulate

        self._bus     = None
        self._running = False
        self._thread  = None
        self._lock    = threading.Lock()

        # State
        self._heading  = 0.0    # degrees, 0 = North
        self._pitch    = 0.0
        self._roll     = 0.0
        self._last_t   = None

    def start(self):
        if not self.simulate:
            try:
                import smbus2
                self._bus = smbus2.SMBus(1)
                # Wake up MPU-6050 (clear sleep bit)
                self._bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
                time.sleep(0.1)
                log.info(f"IMUReader: MPU-6050 @ 0x{self.address:02X} initialised")
            except Exception as e:
                log.error(f"IMUReader: I2C init failed — {e}")
                self.simulate = True

        self._running = True
        target = self._simulate_loop if self.simulate else self._read_loop
        self._thread = threading.Thread(target=target, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)

    def heading(self) -> float:
        """Current yaw heading, 0–360 degrees."""
        with self._lock:
            return self._heading % 360

    def pitch(self) -> float:
        with self._lock:
            return self._pitch

    def roll(self) -> float:
        with self._lock:
            return self._roll

    # ── Internal ──────────────────────────────────────────────

    def _read_raw(self, reg: int) -> int:
        """Read a signed 16-bit value from two consecutive registers."""
        high = self._bus.read_byte_data(self.address, reg)
        low  = self._bus.read_byte_data(self.address, reg + 1)
        val  = (high << 8) | low
        if val >= 0x8000:
            val -= 0x10000
        return val

    def _read_loop(self):
        self._last_t = time.time()
        while self._running:
            try:
                now = time.time()
                dt  = now - self._last_t
                self._last_t = now

                # Gyro (degrees/second)
                gx = self._read_raw(GYRO_XOUT_H)     / GYRO_SCALE
                gy = self._read_raw(GYRO_XOUT_H + 2) / GYRO_SCALE
                gz = self._read_raw(GYRO_XOUT_H + 4) / GYRO_SCALE

                # Accel (g)
                ax = self._read_raw(ACCEL_XOUT_H)     / ACCEL_SCALE
                ay = self._read_raw(ACCEL_XOUT_H + 2) / ACCEL_SCALE
                az = self._read_raw(ACCEL_XOUT_H + 4) / ACCEL_SCALE

                # Accel-derived pitch and roll (stable when static)
                pitch_acc = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))
                roll_acc  = math.degrees(math.atan2(-ax, az))

                # Complementary filter — 98% gyro, 2% accel
                alpha = 0.98
                with self._lock:
                    self._pitch   = alpha * (self._pitch + gy * dt) + (1 - alpha) * pitch_acc
                    self._roll    = alpha * (self._roll  + gx * dt) + (1 - alpha) * roll_acc
                    self._heading = (self._heading + gz * dt) % 360

                time.sleep(0.02)   # 50Hz

            except Exception as e:
                log.error(f"IMUReader: read error — {e}")
                time.sleep(0.5)

    def _simulate_loop(self):
        """Simulates a rover turning slowly through headings."""
        h = 0.0
        while self._running:
            h = (h + 0.5) % 360
            with self._lock:
                self._heading = h
                self._pitch   = 0.0
                self._roll    = 0.0
            time.sleep(0.02)
