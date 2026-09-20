"""Web mission-control console for the Mongla vision stack.

A single ROS2 node (`mission_web`) that serves a browser dashboard: both
cameras side-by-side (video via web_video_server), live detection/vehicle
data over Server-Sent Events, and vision control that writes the SAME ROS
surface the mission DSL writes (SetParameters + latched active_camera) so
the UI and a running DSL mission stay in lock-step.
"""
