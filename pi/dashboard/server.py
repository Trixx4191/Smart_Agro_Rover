"""Compatibility shim that re-exports the dashboard server implementation."""

from pi.vision.dashboard.server import DashboardServer

__all__ = ["DashboardServer"]
