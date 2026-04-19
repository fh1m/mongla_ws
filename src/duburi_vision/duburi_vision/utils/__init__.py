"""CLI utilities for verifying the vision pipeline end-to-end.

Each script is a thin rclpy node that:
  * subscribes to a fixed set of vision topics,
  * prints a clear table of pass/fail health checks,
  * exits 0 on success, 1 on failure -- so they're scriptable for CI.
"""
