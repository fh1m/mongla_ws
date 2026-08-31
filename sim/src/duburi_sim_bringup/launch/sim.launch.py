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
from functools import lru_cache

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
from launch_ros.actions import Node
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

# ArduSub and the ArduPilot Gazebo plugin are built outside this workspace, so
# neither colcon nor the package hooks can point at them.


def _find_root(env_var, marker, candidates, what):
    """Locate an out-of-workspace dependency by looking for a known file.

    Checked in order: the environment variable, then each candidate, and a
    candidate only counts if the marker file is actually there.

    $HOME is the right base even though the dev container points HOME at the
    project root rather than /home/<user>: that is exactly where `stuff/` lives,
    so `$HOME/stuff/ardupilot` expands to the same path the old hardcoded
    absolute fallback spelled out. It was never extra coverage.
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


# $HOME only. The absolute /home/fh1m/... fallbacks that used to sit here were
# one developer's box baked into a launch file: on anyone else's machine they
# just padded the error message with a path that could never exist.
_AP_CANDIDATES = [os.path.join('~', 'stuff', 'ardupilot')]
_AP_GZ_CANDIDATES = [os.path.join('~', 'stuff', 'ardupilot_gazebo')]


@lru_cache(maxsize=None)
def _ardupilot_root():
    return _find_root(
        'ARDUPILOT_ROOT',
        os.path.join('build', 'sitl', 'bin', 'ardusub'),
        _AP_CANDIDATES,
        'the ArduSub SITL build',
    )


@lru_cache(maxsize=None)
def _ardupilot_gazebo_root():
    return _find_root(
        'ARDUPILOT_GAZEBO_ROOT',
        os.path.join('build', 'libArduPilotPlugin.so'),
        _AP_GZ_CANDIDATES,
        'the ArduPilot Gazebo plugin build',
    )


# Resolved lazily, INSIDE generate_launch_description(). As module-level
# constants these raised at import time, so `ros2 launch ... --show-args` -- the
# one command whose entire job is to print arguments without launching anything
# -- died on a box without ArduPilot, and so did every launch file that merely
# included this one. Nothing here needs the paths until a process is spawned.

MAVLINK_PRIMARY_PORT = 14550
# Second link, for MAVProxy or QGroundControl, so attaching a GCS never
# competes with the autonomy stack for the primary one.
MAVLINK_GCS_PORT = 14551

# Somewhere in the Bay of Bengal off Mongla. Only the simulated compass and the
# GPS-denied EKF origin care.
DEFAULT_HOME = '22.4820,89.5860,0.0,0'


def _ardusub_defaults():
    """sub-6dof.parm — sets FRAME_CONFIG 2 (vectored_6dof). Our overlay goes on top."""
    return os.path.join(
        _ardupilot_root(), 'Tools', 'autotest', 'default_params', 'sub-6dof.parm'
    )


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
            'vehicle_model', default_value='duburi_heavy',
            description="The vehicle MODEL name, which is not the course's "
                        'instance name. Both thruster plugins bake their gz '
                        'topics from the model the SDF was generated with.'),
        DeclareLaunchArgument(
            'current', default_value='true',
            description='Run the water-current node (speed 0 = still water).'),
        DeclareLaunchArgument(
            'current_speed', default_value='0.0',
            description='Steady current, m/s. 0.05-0.10 is a realistic pool '
                        'circulation; it is what vision.ki_lat exists to null, '
                        'and with 0 that term has nothing to fight.'),
        DeclareLaunchArgument(
            'current_heading', default_value='0.0',
            description='Current bearing in world frame, degrees.'),
        DeclareLaunchArgument(
            'mavlink_relay', default_value='false',
            description='Route the autopilot link through the fault injector '
                        'so it can be cut (mavlink_loss_s). Off by default: '
                        'with it off, nothing in the fault path touches the '
                        'link the whole session depends on.'),
        DeclareLaunchArgument(
            'scoring', default_value='true',
            description='Score style points and gate transits from ground '
                        'truth. Read-only.'),
        DeclareLaunchArgument(
            'bno085', default_value='true',
            description='Virtual BNO085 board on a PTY, so yaw_source:=bno085 '
                        'and bno085_dvl work in sim.'),
        DeclareLaunchArgument(
            'payload', default_value='true',
            description='Virtual payload board on a PTY, so fire() and the '
                        'mid-hold align(fire=...) shot run in simulation.'),
        DeclareLaunchArgument(
            't200', default_value='true',
            description='Shape thrust through the real T200 curve. Turning '
                        'this OFF leaves the thrusters unfed -- ArduPilotPlugin '
                        'publishes to cmd_thrust_linear and nothing else reads '
                        'it. Off is for A/B comparison only.'),
        DeclareLaunchArgument(
            'battery_volts', default_value='16.0',
            description='Pack voltage the T200 curve is evaluated at. A T200 '
                        'makes 36 N at 12 V and 66 N at 20 V, so this is not '
                        'cosmetic. 16 V is 4S nominal.'),
        DeclareLaunchArgument(
            'thruster_tau', default_value='0.15',
            description='Thruster spin-up time constant, seconds. 0 disables '
                        'the lag and makes thrust step instantly, which is what '
                        'the sim did before.'),
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
            'wash', default_value='false',
            description="Push dynamic props with the hull's wash. OFF by "
                        "default: built and wired, but no run has yet shown it "
                        "moving a prop. See thruster_wash.py."),
        DeclareLaunchArgument(
            'depth_reference', default_value='true',
            description='Zero the simulated depth reference against ground '
                        'truth at startup. Off leaves the raw SITL offset, '
                        'which breaks surface() and mission_reset.'),
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
            os.path.join(_ardupilot_gazebo_root(), 'build')
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
            os.path.join(_ardupilot_root(), 'build', 'sitl', 'bin', 'ardusub'),
            '-w',                       # wipe EEPROM; see the note below
            '-M', 'JSON',               # JSON FDM backend, connects to port 9002
            # Lock-step is negotiated over the JSON link, not on the command
            # line: the plugin's <lock_step>1</lock_step> makes it send
            # "no_lockstep": false and ArduSub follows. The old -S flag for this
            # is gone, and current ArduSub prints "Ignoring stale command-line
            # parameter" rather than failing, so passing it looks harmless and
            # quietly does nothing.
            '--defaults', [_ardusub_defaults(), ',', params_file],
            '-I0',
            '--home', LaunchConfiguration('home'),
            # Primary MAVLink link out to duburi_ws.
            # Straight to the manager, unless the fault injector's relay is
            # in the path -- see the `mavlink_relay` argument.
            '--serial0', [
                'udpclient:127.0.0.1:',
                PythonExpression([
                    "'14559' if '",
                    LaunchConfiguration('mavlink_relay'),
                    "' == 'true' else '", str(MAVLINK_PRIMARY_PORT), "'"]),
            ],
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

    # The T200 curve node. NOT optional in practice: ArduPilotPlugin now
    # publishes to cmd_thrust_LINEAR, and this is the only subscriber -- without
    # it the thrusters get nothing at all and the vehicle sits inert. It is a
    # separate node rather than a plugin because ArduPilotPlugin can only apply
    # an affine map, and a real T200 is not affine.
    t200 = Node(
        package='duburi_sim_bridge',
        executable='t200_curve',
        name='t200_curve',
        parameters=[{
            # The MODEL name, not the course's instance name: both plugins
            # bake their topics from the model the SDF was generated with.
            'vehicle': LaunchConfiguration('vehicle_model'),
            'voltage': LaunchConfiguration('battery_volts'),
            'spinup_tau': LaunchConfiguration('thruster_tau'),
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('t200')),
    )

    # The hull's wash on the props it drives past. `wake_fraction` on the
    # vehicle is NOT this -- that reduces a thruster's own thrust from the
    # hull's own speed and applies no force to anything else. Nothing made a
    # moving hull disturb its surroundings until props became pushable.
    wash = Node(
        package='duburi_sim_bridge',
        executable='thruster_wash',
        name='thruster_wash',
        parameters=[{
            'world': course,
            'enabled': LaunchConfiguration('wash'),
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('wash')),
    )

    # Zero the depth reference at the surface -- the calibration SITL will not
    # do. The vehicle spawns SUBMERGED (each course picks its own z), and
    # ArduSub ACKs PREFLIGHT_CALIBRATION without zeroing, so `AHRS2.altitude`
    # (the depth the stack reads) carries a constant per-course offset: -0.344 m
    # on sauvc26_qualification. That breaks `surface()` (never confirms) and
    # `mission_reset()` (baro re-zero refused). Runs once and exits.
    #
    # It listens on the GCS link, so it is skipped when MAVProxy wants that
    # port -- two readers on one udpin steal each other's packets.
    depth_ref = Node(
        package='duburi_sim_bridge',
        executable='depth_reference',
        name='depth_reference',
        parameters=[{
            'gcs_port': MAVLINK_GCS_PORT,
            'enabled': LaunchConfiguration('depth_reference'),
        }],
        output='screen',
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('ardusub'), "' == 'true' and '",
                LaunchConfiguration('mavproxy'), "' == 'false'"])),
    )

    # Water current. Default 0 m/s so nothing changes unless a course or the
    # operator asks for it -- but the node always runs, so `ros2 param set
    # /water_current speed 0.08` turns the pool on mid-run without a restart.
    current = Node(
        package='duburi_sim_bridge',
        executable='water_current',
        name='water_current',
        parameters=[{
            'vehicle': LaunchConfiguration('vehicle_model'),
            'speed': LaunchConfiguration('current_speed'),
            'heading_deg': LaunchConfiguration('current_heading'),
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('current')),
    )

    # Virtual payload board. Always on: it costs one idle PTY reader, and the
    # alternative is duburi.fire() silently doing nothing in sim -- the exact
    # gap this node exists to close. Point the manager at it with
    # `payload_port:=/tmp/duburi-$USER/payload`.
    payload = Node(
        package='duburi_sim_bridge',
        executable='payload_sim',
        name='payload_sim',
        parameters=[{
            'world': course,
            'vehicle': vehicle_name,
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('payload')),
    )

    # Fault injection. Always on and inert until a fault is armed -- but note
    # it also REPUBLISHES THE DVL (the sensor emits dvl/velocity_raw and this
    # is what turns it into dvl/velocity), so without it every *_dist verb
    # refuses. That is the price of being able to take the DVL away at all.
    faults = Node(
        package='duburi_sim_bridge',
        executable='fault_injection',
        name='faults',
        parameters=[{
            'mavlink_relay': LaunchConfiguration('mavlink_relay'),
            'battery_nominal_v': LaunchConfiguration('battery_volts'),
        }],
        output='screen',
    )

    # Virtual BNO085. Always on: it costs one 50 Hz PTY writer, and with it
    # absent `yaw_source:=bno085` -- what the vehicle actually flies -- cannot
    # be run in sim at all.
    bno = Node(
        package='duburi_sim_bridge',
        executable='bno085_sim',
        name='bno085_sim',
        parameters=[{'vehicle': vehicle_name}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('bno085')),
    )

    # Scoring: style points, gate side, coin flip. Read-only against ground
    # truth, so it can never affect a run it is watching.
    scoring = Node(
        package='duburi_sim_bridge',
        executable='scoring',
        name='scoring',
        parameters=[{'world': course, 'vehicle': vehicle_name}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('scoring')),
    )

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('duburi_sim_bridge'),
                'launch',
                'bridge.launch.py',
            )
        ),
        launch_arguments={
            'vehicle_name': vehicle_name,
            'fx_params': PathJoinSubstitution(
                [worlds_share, 'worlds', [course, '.fx.yaml']]
            ),
        }.items(),
        condition=IfCondition(LaunchConfiguration('bridge')),
    )

    return LaunchDescription(
        args
        + env
        + [
            gazebo_server,
            gazebo_gui,
            t200,
            current,
            payload,
            faults,
            bno,
            scoring,
            wash,
            bridge,
            wait,
            # Everything that talks to the FDM socket starts only once Gazebo has
            # proved it is serving one.
            RegisterEventHandler(
                OnProcessExit(target_action=wait, on_exit=[ardusub, mavproxy, depth_ref])
            ),
        ]
    )
