"""
sensors/battery_monitor.py
──────────────────────────
Tracks rover battery percentage for mission safety + UI.

Sources (first match wins each poll):
  1. Arduino packet field  battery_pct  (0–100)
  2. Arduino packet field  voltage / batt_v  → mapped via config voltages
  3. Time-based estimate from process uptime vs BATTERY_ESTIMATE_HOURS
     (so the dashboard always has a number even without a voltage divider)

Usage:
    from pi.sensors.battery_monitor import BatteryMonitor
    bat = BatteryMonitor(arduino=arduino)
    pct = bat.percent()
    if bat.is_low(): ...
"""

from __future__ import annotations

import time
import logging
from typing import Any, Dict, Optional

log = logging.getLogger(__name__)


class BatteryMonitor:
    def __init__(self, arduino=None):
        from pi.config import (
            BATTERY_LOW_PCT, BATTERY_CRITICAL_PCT,
            BATTERY_FULL_VOLTAGE, BATTERY_EMPTY_VOLTAGE,
            BATTERY_ESTIMATE_HOURS,
        )
        self.arduino = arduino
        self.low_pct = BATTERY_LOW_PCT
        self.critical_pct = BATTERY_CRITICAL_PCT
        self.v_full = BATTERY_FULL_VOLTAGE
        self.v_empty = BATTERY_EMPTY_VOLTAGE
        self.estimate_hours = max(0.25, float(BATTERY_ESTIMATE_HOURS))

        self._started_at = time.time()
        self._last_pct: float = 100.0
        self._last_voltage: Optional[float] = None
        self._source = "estimate"
        self._mission_start: Optional[float] = None

    def start_mission(self):
        """Optional: reset estimate baseline when a mission starts."""
        self._mission_start = time.time()

    def percent(self) -> float:
        """Current battery %, clamped 0–100."""
        pct, voltage, source = self._read()
        self._last_pct = pct
        self._last_voltage = voltage
        self._source = source
        return pct

    def voltage(self) -> Optional[float]:
        self.percent()
        return self._last_voltage

    def source(self) -> str:
        """'arduino_pct' | 'arduino_voltage' | 'estimate'"""
        return self._source

    def is_low(self) -> bool:
        return self.percent() <= self.low_pct

    def is_critical(self) -> bool:
        return self.percent() <= self.critical_pct

    def snapshot(self) -> Dict[str, Any]:
        pct = self.percent()
        return {
            "battery_pct": round(pct, 1),
            "battery_voltage": self._last_voltage,
            "battery_source": self._source,
            "battery_low": pct <= self.low_pct,
            "battery_critical": pct <= self.critical_pct,
        }

    # ── internals ─────────────────────────────────────────────

    def _read(self):
        data = {}
        if self.arduino is not None:
            try:
                data = self.arduino.latest() or {}
            except Exception:
                data = {}

        # 1) Explicit percentage from firmware
        raw_pct = data.get("battery_pct")
        if raw_pct is None:
            raw_pct = data.get("batt_pct")
        if isinstance(raw_pct, (int, float)):
            return float(max(0.0, min(100.0, raw_pct))), self._last_voltage, "arduino_pct"

        # 2) Voltage from firmware → linear map
        raw_v = data.get("voltage")
        if raw_v is None:
            raw_v = data.get("batt_v")
        if isinstance(raw_v, (int, float)) and raw_v > 0:
            pct = self._voltage_to_pct(float(raw_v))
            return pct, float(raw_v), "arduino_voltage"

        # 3) Estimate from wall-clock runtime
        elapsed_h = (time.time() - self._started_at) / 3600.0
        remaining = max(0.0, 1.0 - (elapsed_h / self.estimate_hours))
        pct = round(remaining * 100.0, 1)
        return pct, None, "estimate"

    def _voltage_to_pct(self, volts: float) -> float:
        span = max(0.01, self.v_full - self.v_empty)
        pct = (volts - self.v_empty) / span * 100.0
        return float(max(0.0, min(100.0, pct)))
