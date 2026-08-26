"""Top-level entry point for the Duburi simulator.

    ros2 launch duburi_sim_bringup sim.launch.py course:=sauvc26_qualification

Brings up, in order:

  1. Gazebo Harmonic with the requested course world
  2. a gate that waits for the vehicle IMU to publish
  3. ArduSub SITL, talking to the Gazebo ArduPilot plugin over the JSON backend
  4. the ROS bridge for cameras and ground truth

Then run the autonomy stack against it:

    source ~/Ros_workspaces/duburi_ws/install/setup.zsh
    source install/setup.zsh
    ros2 launch duburi_sim_bringup stack.launch.py vision:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

# ArduSub and the ArduPilot Gazebo plugin are built outside this workspace, so
# neither colcon nor the package hooks can point at them.


def _find_root(env_var, marker, candidates, what):
    """Locate an out-of-workspace dependency by looking for a known file.

    Checked in order: the environment variable, then each candidate. Guessing
    from $HOME is not safe here - the dev container sets HOME to the project
    root rather than to /home/<user> - so candidates are spelled out and each is
    confirmed by the marker file actually being present.
    """
    roots = []
    if env_var in os.environ:
        roots.append(os.environ[env_var])
    roots.extend(candidates)

    for root in roots:
        root = os.path.abspath(os.path.expanduser(root))
        if os.path.exists(os.path.join(root, marker)):
            return root

    raise RuntimeError(
        f'cannot find {what}: no directory containing "{marker}" among '
        f'{roots}. Set {env_var} to its location.'
    )


_AP_CANDIDATES = [
    os.path.join(os.environ.get('HOME', '/'), 'stuff', 'ardupilot'),
    '/home/fh1m/Envs/dockers/auv-ros2/stuff/ardupilot',
]
_AP_GZ_CANDIDATES = [
    os.path.join(os.environ.get('HOME', '/'), 'stuff', 'ardupilot_gazebo'),
    '/home/fh1m/Envs/dockers/auv-ros2/stuff/ardupilot_gazebo',
]

ARDUPILOT_ROOT = _find_root(
    'ARDUPILOT_ROOT',
    os.path.join('build', 'sitl', 'bin', 'ardusub'),
    _AP_CANDIDATES,
    'the ArduSub SITL build',
)
ARDUPILOT_GAZEBO_ROOT = _find_root(
    'ARDUPILOT_GAZEBO_ROOT',
    os.path.join('build', 'libArduPilotPlugin.so'),
    _AP_GZ_CANDIDATES,
    'the ArduPilot Gazebo plugin build',
)

ARDUSUB_BIN = os.path.join(ARDUPILOT_ROOT, 'build', 'sitl', 'bin', 'ardusub')
# Sets FRAME_CONFIG 2 (vectored_6dof). Our overlay is applied on top.
ARDUPILOT_SUB_DEFAULTS = os.path.join(
    ARDUPILOT_ROOT, 'Tools', 'autotest', 'default_params', 'sub-6dof.parm'
)

# duburi_ws connects with udpin:0.0.0.0:14550, so ArduSub has to push to it.
MAVLINK_PRIMARY_PORT = 14550
# Second link, for MAVProxy or QGroundControl, so attaching a GCS never
# competes with the autonomy stack for the primary one.
MAVLINK_GCS_PORT = 14551

# Somewhere in the Bay of Bengal off Mongla. Only the simulated compass and the
# GPS-denied EKF origin care.
DEFAULT_HOME = '22.4820,89.5860,0.0,0'


def generate_launch_description():
    bringup_share = get_package_share_directory('duburi_sim_bringup')
    worlds_share = get_package_share_directory('duburi_sim_worlds')

    course = LaunchConfiguration('course')
    vehicle_name = LaunchConfiguration('vehicle_name')
    gui = LaunchConfiguration('gui')
    verbose = LaunchConfiguration('verbose')

    world_file = PathJoinSubstitution([worlds_share, 'worlds', [course, '.world']])
    gui_config = PathJoinSubstitution([bringup_share, 'config', 'gui.config'])
    params_file = PathJoinSubstitution(
        [bringup_share, 'config', LaunchConfiguration('ardusub_params')]
    )
    wait_script = os.path.join(bringup_share, 'launch', 'wait_for_gazebo.py')

    args = [
        DeclareLaunchArgument(
            'course',
            default_value='sauvc26_qualification',
            description='Course world in duburi_sim_worlds/worlds. '
                        'Run `gen_world.py --list` to see the options.',
        ),
        DeclareLaunchArgument(
            'vehicle_name',
            default_value='duburi',
            description='Vehicle instance name, must match the course YAML.',
        ),
        DeclareLaunchArgument(
            'ardusub', default_value='true',
            description='Run ArduSub SITL. Set false to drive the thrusters '
                        'directly with `gz topic` for hydrodynamic tuning.',
        ),
        DeclareLaunchArgument(
            'bridge', default_value='true',
            description='Run the ROS bridge for cameras and ground truth.',
        ),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Run the Gazebo GUI.'),
        DeclareLaunchArgument(
            'mavproxy', default_value='false',
            description=f'Open a MAVProxy console on UDP {MAVLINK_GCS_PORT}.',
        ),
        DeclareLaunchArgument(
            'ardusub_params', default_value='duburi_sub.parm',
            description='Parameter overlay in duburi_sim_bringup/config. Use '
                        'duburi_sub_extnav.parm to test the ATT_POS_MOCAP path.',
        ),
        DeclareLaunchArgument('home', default_value=DEFAULT_HOME,
                              description='ArduSub home as lat,lon,alt,heading.'),
        DeclareLaunchArgument('verbose', default_value='2',
                              description='Gazebo verbosity, 0 to 4.'),
    ]

    # The ArduPilot plugin lives outside this workspace, so no colcon hook can
    # put it on the path.
    env = [
        SetEnvironmentVariable(
            'GZ_SIM_SYSTEM_PLUGIN_PATH',
            os.path.join(ARDUPILOT_GAZEBO_ROOT, 'build')
            + os.pathsep
            + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
        ),
        # Pin gz-transport to loopback. Every process in this simulator is on
        # one host, and a dev box with wifi plus docker bridges gives
        # gz-transport several interfaces to choose between; left to itself its
        # discovery becomes slow and, worse, partial. Partial discovery does not
        # raise anything - individual topics just never connect and the commands
        # published on them are dropped - so it presents as a physics bug.
        SetEnvironmentVariable('GZ_IP', os.environ.get('GZ_IP', '127.0.0.1')),
    ]

    # Server always runs. When gui:=true we also start a separate GUI client so a
    # Qt/X11 failure cannot take down the physics/sensors process (combined
    # `gz sim` aborts the whole binary if the GUI cannot open the display).
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-v', verbose, '-s', '-r', world_file],
        output='screen',
    )
    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-v', verbose, '-g',
             '--gui-config', gui_config],
        output='screen',
        condition=IfCondition(gui),
    )

    # Gate ArduSub on the vehicle IMU actually publishing. See wait_for_gazebo.py
    # for why a fixed sleep is not good enough.
    wait = ExecuteProcess(
        cmd=['python3', wait_script, '--world', course, '--model', vehicle_name],
        output='screen',
    )

    ardusub = ExecuteProcess(
        cmd=[
            ARDUSUB_BIN,
            '-w',                       # wipe EEPROM; see the note below
            '-M', 'JSON',               # JSON FDM backend, connects to port 9002
            # Lock-step is negotiated over the JSON link, not on the command
            # line: the plugin's <lock_step>1</lock_step> makes it send
            # "no_lockstep": false and ArduSub follows. The old -S flag for this
            # is gone, and current ArduSub prints "Ignoring stale command-line
            # parameter" rather than failing, so passing it looks harmless and
            # quietly does nothing.
            '--defaults', [ARDUPILOT_SUB_DEFAULTS, ',', params_file],
            '-I0',
            '--home', LaunchConfiguration('home'),
            # Primary MAVLink link out to duburi_ws.
            '--serial0', f'udpclient:127.0.0.1:{MAVLINK_PRIMARY_PORT}',
            # Secondary link for a GCS.
            '--serial1', f'udpclient:127.0.0.1:{MAVLINK_GCS_PORT}',
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('ardusub')),
        # -w wipes stored parameters on every start. Keep it: ArduSub caches the
        # thruster mapping, and switching between `vectored` and `vectored_6dof`
        # without a wipe leaves thruster 8 permanently unresponsive.
    )

    mavproxy = ExecuteProcess(
        cmd=['mavproxy.py', '--master', f'udp:127.0.0.1:{MAVLINK_GCS_PORT}',
             '--console'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('mavproxy')),
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('duburi_sim_bridge'),
                'launch',
                'bridge.launch.py',
            )
        ),
        launch_arguments={'vehicle_name': vehicle_name}.items(),
        condition=IfCondition(LaunchConfiguration('bridge')),
    )

    return LaunchDescription(
        args
        + env
        + [
            gazebo_server,
            gazebo_gui,
            bridge,
            wait,
            # Everything that talks to the FDM socket starts only once Gazebo has
            # proved it is serving one.
            RegisterEventHandler(
                OnProcessExit(target_action=wait, on_exit=[ardusub, mavproxy])
            ),
        ]
    )
