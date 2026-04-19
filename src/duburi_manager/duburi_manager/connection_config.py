#!/usr/bin/env python3
"""
Connection + network profiles for the duburi_manager node.

AUV Ethernet topology (test platform: Duburi 4.2)
-------------------------------------------------

    Pixhawk 2.4.8 ── USB ──► Raspberry Pi (BlueOS)   192.168.2.1  / GW 192.168.2.2
                                    │
                                    │  switch
                                    ▼
                              Jetson Orin Nano       192.168.2.69  (static)
                                    ▲
                                    │  UDP 14550   (BlueOS 'inspector' endpoint,
                              ros2 stack                UDP Client → Jetson:14550)

ROS2 side always listens on ``udpin:0.0.0.0:14550`` — the same line works
in sim, desk-over-ethernet, and pool modes because BlueOS pushes MAVLink
at us; we never dial out. For desk mode (Pixhawk plugged directly via
USB) we fall back to the serial device.

BlueOS endpoint config (web UI → Vehicle → Pixhawk → Endpoints)::

    Name:   inspector
    Type:   UDP Client
    IP:     192.168.2.69         (Jetson static IP)
    Port:   14550
"""

NETWORK = {
    'jetson_ip': '192.168.2.69',    # static IP on switch
    'blueos_ip': '192.168.2.1',     # Raspberry Pi hosting BlueOS
    'blueos_gw': '192.168.2.2',     # BlueOS gateway
    'mav_port':  14550,             # MAVLink inspector endpoint port
    'endpoint':  'inspector',       # BlueOS endpoint name (UDP Client)
}

PROFILES = {
    'sim':    {'conn': 'udpin:0.0.0.0:14550', 'baud': None},   # Docker + ArduSub SITL
    'pool':   {'conn': 'udpin:0.0.0.0:14550', 'baud': None},   # Jetson on pool deck
    'laptop': {'conn': 'udpin:0.0.0.0:14550', 'baud': None},   # Laptop on same switch
    'desk':   {'conn': '/dev/ttyACM0',        'baud': 115200}, # Pixhawk over USB
}

DEFAULT_MODE = 'sim'
