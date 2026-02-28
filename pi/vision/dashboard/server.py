"""
dashboard/server.py
────────────────────
Lightweight Flask web dashboard.
Serves a local web page showing live sensor data, trends, and crop images.
Accessible on any device on the same network as the Pi.

URL: http://<pi-ip>:5000
"""

import os
import json
import logging
from datetime import datetime

log = logging.getLogger(__name__)


class DashboardServer:

    def __init__(self, logger):
        self.logger = logger

    def run(self, debug: bool = False):
        from pi.config import DASHBOARD_HOST, DASHBOARD_PORT
        try:
            from flask import Flask, jsonify, render_template_string, send_from_directory
        except ImportError:
            log.error("Flask not installed — dashboard unavailable")
            return

        app = Flask(__name__)
        logger = self.logger

        # ── HTML template (single-file, no external files needed) ──
        HTML = """
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta http-equiv="refresh" content="10">
  <title>Farm Rover Dashboard</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; }
    body { font-family: Arial, sans-serif; background: #f0f4f0; color: #222; }
    header { background: #1A5276; color: white; padding: 16px 24px; }
    header h1 { font-size: 1.4rem; }
    header p  { font-size: 0.85rem; opacity: 0.8; margin-top: 4px; }
    .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr));
            gap: 16px; padding: 20px; }
    .card { background: white; border-radius: 8px; padding: 18px;
            box-shadow: 0 2px 6px rgba(0,0,0,0.08); }
    .card h3 { font-size: 0.75rem; color: #888; text-transform: uppercase;
               letter-spacing: 0.05em; margin-bottom: 8px; }
    .value { font-size: 2rem; font-weight: bold; color: #1A5276; }
    .unit  { font-size: 0.9rem; color: #888; }
    .stress { background: #fff3cd; border-left: 4px solid #e67e22; padding: 12px 16px;
              margin: 0 20px 20px; border-radius: 4px; font-size: 0.9rem; }
    .ok     { background: #d5f5e3; border-left: 4px solid #1E8449; }
    table  { width: 100%; border-collapse: collapse; font-size: 0.85rem; margin: 0 20px 20px;
             width: calc(100% - 40px); background: white; border-radius: 8px; overflow: hidden;
             box-shadow: 0 2px 6px rgba(0,0,0,0.08); }
    th { background: #1E8449; color: white; padding: 10px 12px; text-align: left; }
    td { padding: 9px 12px; border-bottom: 1px solid #eee; }
    tr:last-child td { border-bottom: none; }
    tr:nth-child(even) td { background: #f9fdf9; }
    .images { display: flex; flex-wrap: wrap; gap: 12px; padding: 0 20px 20px; }
    .images img { width: 200px; height: 150px; object-fit: cover;
                  border-radius: 6px; box-shadow: 0 2px 6px rgba(0,0,0,0.12); }
    footer { text-align: center; padding: 16px; color: #999; font-size: 0.8rem; }
  </style>
</head>
<body>
<header>
  <h1>🌱 Autonomous Farm Rover — Dashboard</h1>
  <p>Auto-refreshes every 10 seconds &nbsp;|&nbsp; {{ now }}</p>
</header>

<div class="grid">
  <div class="card">
    <h3>Soil Moisture 1</h3>
    <div class="value">{{ summary.avg_moisture_1 | round(1) }}<span class="unit">%</span></div>
  </div>
  <div class="card">
    <h3>Soil Moisture 2</h3>
    <div class="value">{{ summary.avg_moisture_2 | round(1) }}<span class="unit">%</span></div>
  </div>
  <div class="card">
    <h3>Soil Temperature</h3>
    <div class="value">{{ summary.avg_soil_temp | round(1) }}<span class="unit">°C</span></div>
  </div>
  <div class="card">
    <h3>Air Temperature</h3>
    <div class="value">{{ summary.avg_air_temp | round(1) }}<span class="unit">°C</span></div>
  </div>
  <div class="card">
    <h3>Humidity</h3>
    <div class="value">{{ summary.avg_humidity | round(1) }}<span class="unit">%</span></div>
  </div>
  <div class="card">
    <h3>Total Readings</h3>
    <div class="value">{{ summary.total_readings }}</div>
  </div>
</div>

{% if summary.stress_events > 0 %}
<div class="stress">
  ⚠️ <strong>{{ summary.stress_events }} stress event(s) detected</strong> — check crop images below
</div>
{% else %}
<div class="stress ok">
  ✅ No crop stress detected in current session
</div>
{% endif %}

<h2 style="padding: 0 20px 12px; color: #1A5276; font-size: 1rem;">Recent Readings</h2>
<table>
  <thead>
    <tr>
      <th>Time</th>
      <th>Moisture 1</th>
      <th>Moisture 2</th>
      <th>Soil °C</th>
      <th>Air °C</th>
      <th>Humidity</th>
      <th>GPS</th>
      <th>Stress</th>
    </tr>
  </thead>
  <tbody>
    {% for r in readings %}
    <tr>
      <td>{{ r.datetime }}</td>
      <td>{{ r.moisture_1 }}%</td>
      <td>{{ r.moisture_2 }}%</td>
      <td>{{ r.soil_temp_1 }}°C</td>
      <td>{{ r.air_temp }}°C</td>
      <td>{{ r.humidity }}%</td>
      <td>{% if r.latitude %}{{ "%.5f"|format(r.latitude) }}, {{ "%.5f"|format(r.longitude) }}{% else %}—{% endif %}</td>
      <td>{% if r.stress_flag %}⚠️{% else %}✅{% endif %}</td>
    </tr>
    {% endfor %}
  </tbody>
</table>

{% if images %}
<h2 style="padding: 0 20px 12px; color: #1A5276; font-size: 1rem;">Crop Images</h2>
<div class="images">
  {% for img in images %}
    <img src="/images/{{ img }}" alt="{{ img }}" title="{{ img }}">
  {% endfor %}
</div>
{% endif %}

<footer>Farm Rover v1.0 &nbsp;|&nbsp; Local Dashboard &nbsp;|&nbsp; No internet required</footer>
</body>
</html>
"""

        @app.route("/")
        def index():
            summary  = logger.summary()
            readings = logger.get_last(20)
            # fill None values for template safety
            for k in ["avg_moisture_1","avg_moisture_2","avg_soil_temp","avg_air_temp","avg_humidity"]:
                if summary.get(k) is None:
                    summary[k] = 0.0
            if summary.get("total_readings") is None:
                summary["total_readings"] = 0
            if summary.get("stress_events") is None:
                summary["stress_events"] = 0

            from pi.config import IMAGE_DIR
            try:
                images = [f for f in os.listdir(IMAGE_DIR)
                          if f.endswith(".jpg") and "annotated" not in f]
                images.sort(reverse=True)
                images = images[:12]
            except Exception:
                images = []

            now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            return render_template_string(HTML,
                summary=summary, readings=readings, images=images, now=now)

        @app.route("/api/summary")
        def api_summary():
            return jsonify(logger.summary())

        @app.route("/api/readings")
        def api_readings():
            return jsonify(logger.get_last(50))

        @app.route("/api/stress")
        def api_stress():
            return jsonify(logger.get_stress_events())

        @app.route("/images/<path:filename>")
        def serve_image(filename):
            from pi.config import IMAGE_DIR
            return send_from_directory(IMAGE_DIR, filename)

        log.info(f"Dashboard starting at http://{DASHBOARD_HOST}:{DASHBOARD_PORT}")
        app.run(host=DASHBOARD_HOST, port=DASHBOARD_PORT,
                debug=debug, use_reloader=False)
