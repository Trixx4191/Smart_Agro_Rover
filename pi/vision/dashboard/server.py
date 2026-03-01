"""
dashboard/server.py
────────────────────
Flask dashboard server — serves the new production UI.
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
            from flask import Flask, jsonify, send_from_directory, render_template
        except ImportError:
            log.error("Flask not installed — dashboard unavailable")
            return

        # Point Flask at the templates folder next to this file
        template_dir = os.path.join(os.path.dirname(__file__), "templates")
        app = Flask(__name__, template_folder=template_dir)
        logger = self.logger

        @app.route("/")
        def index():
            return render_template("dashboard.html")

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

        log.info(f"Dashboard: http://{DASHBOARD_HOST}:{DASHBOARD_PORT}")
        app.run(host=DASHBOARD_HOST, port=DASHBOARD_PORT,
                debug=debug, use_reloader=False)
