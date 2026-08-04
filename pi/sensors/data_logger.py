"""
sensors/data_logger.py
──────────────────────
Logs every sensor reading to:
  1. CSV  — one file per session  (human-readable, easy to share)
  2. SQLite — persistent database (for dashboard queries)

Usage:
    from pi.sensors.data_logger import DataLogger

    logger = DataLogger()
    logger.log(sensor_data, gps_coords=(lat, lon), image_path="data/images/wp1.jpg")
    df = logger.get_recent(hours=1)
    logger.close()
"""

import csv
import sqlite3
import os
import time
import logging
from datetime import datetime
from typing import Optional, Tuple, Dict, Any

log = logging.getLogger(__name__)


class DataLogger:

    def __init__(self, db_path: str = None, log_dir: str = None):
        from pi.config import DB_PATH, LOG_DIR
        self.db_path  = db_path  or DB_PATH
        self.log_dir  = log_dir  or LOG_DIR

        os.makedirs(self.log_dir, exist_ok=True)
        os.makedirs(os.path.dirname(self.db_path), exist_ok=True)

        # One CSV file per session
        session_ts   = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(self.log_dir, f"session_{session_ts}.csv")

        self._csv_file   = None
        self._csv_writer = None
        self._db_conn    = None

        self._init_csv()
        self._init_db()

    # ── Initialisation ────────────────────────────────────────

    def _init_csv(self):
        self._csv_file = open(self.csv_path, "w", newline="")
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=[
            "timestamp", "datetime",
            "moisture_1", "moisture_2",
            "soil_temp_1", "soil_temp_2",
            "air_temp", "humidity", "pressure",
            "us_front", "us_rear",
            "enc_lf", "enc_lr", "enc_rf", "enc_rr",
            "latitude", "longitude",
            "image_path",
            "stress_flag", "stress_score",
        ])
        self._csv_writer.writeheader()
        self._csv_file.flush()
        log.info(f"DataLogger: CSV → {self.csv_path}")

    def _init_db(self):
        self._db_conn = sqlite3.connect(self.db_path, check_same_thread=False)
        self._db_conn.execute("""
            CREATE TABLE IF NOT EXISTS readings (
                id           INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp    REAL,
                datetime     TEXT,
                moisture_1   INTEGER,
                moisture_2   INTEGER,
                soil_temp_1  REAL,
                soil_temp_2  REAL,
                air_temp     REAL,
                humidity     REAL,
                pressure     REAL,
                us_front     INTEGER,
                us_rear      INTEGER,
                enc_lf       INTEGER,
                enc_lr       INTEGER,
                enc_rf       INTEGER,
                enc_rr       INTEGER,
                latitude     REAL,
                longitude    REAL,
                image_path   TEXT,
                stress_flag  INTEGER DEFAULT 0,
                stress_score REAL    DEFAULT 0.0
            )
        """)
        self._db_conn.execute("""
            CREATE INDEX IF NOT EXISTS idx_timestamp ON readings(timestamp)
        """)
        self._db_conn.commit()
        log.info(f"DataLogger: SQLite → {self.db_path}")

    # ── Core log method ───────────────────────────────────────

    def log(
        self,
        sensor_data: Dict[str, Any],
        gps_coords: Optional[Tuple[float, float]] = None,
        image_path: Optional[str] = None,
        stress_flag: bool = False,
        stress_score: float = 0.0,
    ):
        """
        Write a reading to both CSV and SQLite.

        Args:
            sensor_data  : packet from SerialHandler.latest()
            gps_coords   : (latitude, longitude) or None
            image_path   : relative path to saved crop image
            stress_flag  : True if vision detected stress
            stress_score : 0.0–1.0 confidence of stress detection
        """
        ts  = time.time()
        dt  = datetime.fromtimestamp(ts).strftime("%Y-%m-%d %H:%M:%S")
        lat = gps_coords[0] if gps_coords else None
        lon = gps_coords[1] if gps_coords else None

        moisture  = sensor_data.get("moisture")  or [None, None]
        soil_temp = sensor_data.get("soil_temp") or [None, None]
        enc       = sensor_data.get("enc")        or [0, 0, 0, 0]

        row = {
            "timestamp":    ts,
            "datetime":     dt,
            "moisture_1":   moisture[0]  if len(moisture)  > 0 else None,
            "moisture_2":   moisture[1]  if len(moisture)  > 1 else None,
            "soil_temp_1":  soil_temp[0] if len(soil_temp) > 0 else None,
            "soil_temp_2":  soil_temp[1] if len(soil_temp) > 1 else None,
            "air_temp":     sensor_data.get("air_temp"),
            "humidity":     sensor_data.get("humidity"),
            "pressure":     sensor_data.get("pressure"),
            "us_front":     sensor_data.get("us_front"),
            "us_rear":      sensor_data.get("us_rear"),
            "enc_lf":       enc[0] if len(enc) > 0 else 0,
            "enc_lr":       enc[1] if len(enc) > 1 else 0,
            "enc_rf":       enc[2] if len(enc) > 2 else 0,
            "enc_rr":       enc[3] if len(enc) > 3 else 0,
            "latitude":     lat,
            "longitude":    lon,
            "image_path":   image_path,
            "stress_flag":  1 if stress_flag else 0,
            "stress_score": round(stress_score, 4),
        }

        # Write to CSV
        try:
            self._csv_writer.writerow(row)
            self._csv_file.flush()
        except Exception as e:
            log.error(f"DataLogger: CSV write error — {e}")

        # Write to SQLite
        try:
            self._db_conn.execute("""
                INSERT INTO readings (
                    timestamp, datetime,
                    moisture_1, moisture_2,
                    soil_temp_1, soil_temp_2,
                    air_temp, humidity, pressure,
                    us_front, us_rear,
                    enc_lf, enc_lr, enc_rf, enc_rr,
                    latitude, longitude,
                    image_path, stress_flag, stress_score
                ) VALUES (
                    :timestamp, :datetime,
                    :moisture_1, :moisture_2,
                    :soil_temp_1, :soil_temp_2,
                    :air_temp, :humidity, :pressure,
                    :us_front, :us_rear,
                    :enc_lf, :enc_lr, :enc_rf, :enc_rr,
                    :latitude, :longitude,
                    :image_path, :stress_flag, :stress_score
                )
            """, row)
            self._db_conn.commit()
        except Exception as e:
            log.error(f"DataLogger: SQLite write error — {e}")

        log.debug(f"Logged: moisture={row['moisture_1']}/{row['moisture_2']}% "
                  f"soil={row['soil_temp_1']}°C air={row['air_temp']}°C "
                  f"gps=({lat},{lon})")

    # ── Query methods ─────────────────────────────────────────

    def get_recent(self, hours: float = 1.0) -> list:
        """Return all readings from the last N hours as list of dicts."""
        since = time.time() - (hours * 3600)
        cur = self._db_conn.execute(
            "SELECT * FROM readings WHERE timestamp >= ? ORDER BY timestamp DESC",
            (since,)
        )
        cols = [d[0] for d in cur.description]
        return [dict(zip(cols, row)) for row in cur.fetchall()]

    def get_last(self, n: int = 10) -> list:
        """Return the last N readings."""
        cur = self._db_conn.execute(
            "SELECT * FROM readings ORDER BY timestamp DESC LIMIT ?", (n,)
        )
        cols = [d[0] for d in cur.description]
        return [dict(zip(cols, row)) for row in cur.fetchall()]

    def get_stress_events(self) -> list:
        """Return all readings where stress was detected."""
        cur = self._db_conn.execute(
            "SELECT * FROM readings WHERE stress_flag=1 ORDER BY timestamp DESC"
        )
        cols = [d[0] for d in cur.description]
        return [dict(zip(cols, row)) for row in cur.fetchall()]

    def summary(self) -> Dict[str, Any]:
        """Return quick stats for the dashboard."""
        cur = self._db_conn.execute("""
            SELECT
                COUNT(*)           AS total_readings,
                AVG(moisture_1)    AS avg_moisture_1,
                AVG(moisture_2)    AS avg_moisture_2,
                AVG(soil_temp_1)   AS avg_soil_temp,
                AVG(air_temp)      AS avg_air_temp,
                AVG(humidity)      AS avg_humidity,
                SUM(stress_flag)   AS stress_events
            FROM readings
        """)
        cols = [d[0] for d in cur.description]
        row  = cur.fetchone()
        return dict(zip(cols, row)) if row else {}

    # ── Cleanup ───────────────────────────────────────────────

    def close(self):
        if self._csv_file:
            self._csv_file.close()
        if self._db_conn:
            self._db_conn.close()
        log.info("DataLogger: closed")
