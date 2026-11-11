"""Batighor, the operator center: the bridge, the web app, the overlay and the helm.

    ros2 launch mongla_manager bringup.launch.py vision:=true batighor:=true   # the vehicle
    ros2 launch mongla_batighor batighor.launch.py                             # beside any stack

bringup includes this file. Alone, it runs beside a vision launch with no board
and no manager, which is how a bench watches cameras or a recorded video:

    ros2 launch mongla_vision vision_dual.launch.py fwd_video:=/path/gate.mp4 dwn_video:=/path/bin.mp4
    ros2 launch mongla_batighor batighor.launch.py

Open http://<host>:<port>/ in any browser. Every process restarts after it dies,
as bringup's do (`launch_respawn`).
"""
from functools import partial

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from mongla_localization.launch_respawn import respawning

# What a browser may do through foxglove_bridge. The bridge cannot tell one
# browser from another, and its defaults let every one of them call any
# service, set any node's parameters and publish on any topic -- which with
# include_hidden reaches /mongla/move/_action/send_goal and
# shield_max_depth_m, past the helm. So a browser calls the helm and nothing
# else: it drives, tunes the detectors (not during a mission, owner
# 2026-11-12) and switches the camera there, by the helm's rules
# (mongla_batighor/seat.py), and it publishes nothing.
# test_the_bridge_only_lets_a_browser_drive_through_the_helm.
BRIDGE_LIMITS = {
    'capabilities': ['services', 'connectionGraph', 'assets'],
    'service_whitelist': ['^/mongla/batighor/helm$'],
    'param_whitelist': ['^$'],
    'client_topic_whitelist': ['^$'],
}


def generate_launch_description():
    args = [
        DeclareLaunchArgument('batighor_port', default_value='8000',
                              description='Batighor web port'),
        DeclareLaunchArgument('foxglove_port', default_value='8765',
                              description='foxglove_bridge WebSocket port'),
    ]
    # The bridge exposes every topic over a WebSocket that Lichtblick, in the
    # browser or on the desktop, renders. use_compression cuts tether
    # bandwidth; still confirm detection FPS is unperturbed with it up and
    # images being viewed (Mongla is FPS-coupled). See foxglove-and-bags.md.
    bridge = partial(
        Node, package='foxglove_bridge', executable='foxglove_bridge',
        name='foxglove_bridge', output='screen',
        ros_arguments=['--log-level', 'warn', '--log-level', 'foxglove_bridge:=info'],
        parameters=[{
            'port':            LaunchConfiguration('foxglove_port'),
            'address':         '0.0.0.0',
            'use_compression': True,
            # /mongla/move/_action/feedback + /status live under the _action
            # namespace, which ROS 2 hides. Without this the err_x_px/err_y_px
            # convergence plot is silently empty (the default is false).
            'include_hidden':  True,
            **BRIDGE_LIMITS,
        }])
    serve = partial(
        Node, package='mongla_batighor', executable='serve', name='batighor',
        output='screen',
        arguments=['--port', LaunchConfiguration('batighor_port'),
                   '--bridge-port', LaunchConfiguration('foxglove_port')])
    # The detector's boxes as annotations the browser draws over the camera's
    # own JPEG (overlay.py). Publishes only while watched.
    overlay = partial(Node, package='mongla_batighor', executable='overlay',
                      name='batighor_overlay', output='screen')
    # The browsers' helm: the pilot token, tuning, the camera and the command
    # log (helm.py). The terminal does not need it.
    helm = partial(Node, package='mongla_batighor', executable='helm',
                   name='batighor_helm', output='screen')
    return LaunchDescription(args + [
        *respawning(bridge, 'the Foxglove bridge'),
        *respawning(serve, 'the Batighor web app'),
        *respawning(overlay, 'the Batighor overlay'),
        *respawning(helm, 'the Batighor helm'),
    ])
