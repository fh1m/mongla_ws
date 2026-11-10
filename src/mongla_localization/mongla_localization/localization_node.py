"""The vehicle's own answer to "where am I", fused from every sensor it has.

Why this node exists at all. `inekf.py` landed as a tested library with no
caller -- 19 unit tests, a converging filter, and nothing on the vehicle ever
constructing one. That is the failure the bumblebee doctrine names directly:
effort is not points, and a module nothing calls scores zero however good it
is. This file is the wiring, and it is deliberately thin. Every decision worth
arguing about lives in `inekf.py` and is tested without ROS.

THE SPLIT, which is the point. The SROT board runs a 500 Hz attitude loop with
the BNO085 in hand and owns every inner loop. It is not trying to know where it
is in the pool, and it has neither the cameras nor the memory to. This node is
the outer half: it takes the board's inertial stream, the downward camera's
optical flow, the barometer, and pool-frame position fixes resected off props,
and produces one pose. Board fuses what is fast; Pi fuses what is wide.

TIME. Every board-sourced sample is stamped through `flow_timing.ClockMap` in
the manager, not on arrival -- measured on this vehicle, the board's ATTITUDE
interval has sd 0.00 ms where host arrival has sd 6.67 ms and p2p 35.12. This
node reads `header.stamp` and never `now()`, or it would put all of that
jitter back into dt.

No IMU, no predict. There is deliberately no synthetic propagation when the
inertial stream stops. A filter that holds its state and keeps taking depth,
yaw and flow updates is honest about having no new inertial information; one
that invents `a = 0` to keep a timer fed is dead-reckoning on an assumption
and says nothing about it. Measured 2026-09-11: after a bad boot the board
streamed ATTITUDE and SCALED_IMU2 as exact 0.0 in every field while reporting
3D_GYRO and 3D_ACCEL unhealthy -- so "zeros arrived" and "the hull is still"
are the same bytes, and only the health bit separates them. `SrotFC.get_imu`
returns None in that state, no message is published, and this node coasts.

Where each part lives. `LocalizationNode` is one class in three files:

  localization_node.py     parameters, subscriptions, timers, main()
  localization_inputs.py   one callback per measurement, and the stillness and
                           motion-model checks they feed
  localization_publish.py  /mongla/odom, its covariance, the diagnostic line
"""
from __future__ import annotations

from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from geometry_msgs.msg import (PointStamped, TwistWithCovarianceStamped,
                               Vector3Stamped)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String

from mongla_interfaces.msg import FloorHeading, MonglaState

from mongla_localization.command_velocity import (CommandVelocityModel,
                                                  MotionCheck)
from mongla_localization import frames
from mongla_localization.inekf import RIEKF
from mongla_localization.retro import Retrodictor

from .localization_inputs import STILL_WINDOW, LocalizationInputsMixin
from .localization_publish import LocalizationPublishMixin


class LocalizationNode(LocalizationInputsMixin, LocalizationPublishMixin, Node):
    def __init__(self, **kw):
        super().__init__('mongla_localization', **kw)

        self._filter = RIEKF()
        self._last_imu_t: float | None = None
        self._last_accel = None          # body specific force, latest IMU
        self._imu_gap_warned = False
        # Counters, published in the diagnostic line. Which channel is feeding
        # the filter is the first question when a pose looks wrong, and a rate
        # of zero on one input is invisible in the pose itself.
        self._n = {'imu': 0, 'att': 0, 'depth': 0, 'yaw': 0, 'flow': 0,
                   'flow_refused': 0, 'depth_refused': 0, 'yaw_refused': 0,
                   'fix': 0, 'zupt': 0, 'grid': 0, 'grid_refused': 0,
                   'lane': 0, 'lane_refused': 0, 'model': 0, 'gap': 0,
                   # World-frame updates the observability gate refused. A
                   # non-zero count is not an error -- it is the gate doing its
                   # job -- but a count that RISES while flow is healthy means
                   # the threshold is wrong for this vehicle. See B-56.
                   'gated': 0}
        # Whether the OBSERVABILITY GATE is open, latched so it is published
        # only on change. See `_report_aiding`.
        self._aiding_state = None
        self._still: deque = deque(maxlen=STILL_WINDOW)
        self._last_flow_t = 0.0
        self._attitude_seeded = False
        self._anchored = False
        # Board yaw + this = pool yaw. Zero until an anchor arrives. Every
        # board attitude (and board yaw, under `use_yaw`) is rotated by it
        # before it reaches the filter -- see `_on_heading` for why the anchor
        # has to live HERE and not as a one-off filter update.
        self._yaw_offset_deg = 0.0
        # The latest RAW board attitude, the partner the anchor is paired with.
        self._board_R = None
        self._last_input_t = 0.0
        self._aided_at_last_diag = -1

        cam = str(self.declare_parameter('flow_camera', 'downward').value)
        self._flow_sigma = float(self.declare_parameter('flow_sigma', 0.05).value)
        # The downward camera is not at the IMU (issue #27). `None` until it
        # is taped on the built hull -- and None means UNCORRECTED, exactly the
        # behaviour before this existed, reported rather than hidden. It is not
        # a zero standing in for a measurement.
        self._flow_lever_arm = frames.load().downward_cam
        if self._flow_lever_arm is None:
            self.get_logger().warning(
                '[LOCAL] downward camera lever arm NOT MEASURED (frames.yaml '
                'imu_to.downward_cam is null): flow is fused WITHOUT omega x r, '
                'so every turn in place is read as sideways motion -- 0.125 m/s '
                'at 0.5 rad/s for a lens 0.25 m from the IMU. Tape it.')
        self._depth_sigma = float(self.declare_parameter('depth_sigma', 0.02).value)
        self._yaw_sigma_deg = float(self.declare_parameter('yaw_sigma_deg', 2.0).value)
        self._fix_sigma = float(self.declare_parameter('fix_sigma', 0.5).value)
        # The BNO's datasheet drift, not a tuning guess: under 0.01 deg/min at
        # rest, measured on this board over 8 minutes. Loose enough that a
        # manoeuvre's transient does not fight the filter, tight enough that
        # attitude is effectively pinned to the board.
        self._attitude_sigma_deg = float(
            self.declare_parameter('attitude_sigma_deg', 0.5).value)
        self._zupt_sigma = float(self.declare_parameter('zupt_sigma', 0.01).value)
        self._zupt_enabled = bool(self.declare_parameter('zupt', True).value)
        # The grid is geometry, not a compass: when it applies it is very
        # precise (measured +/-1.0 deg against known truth), so a tight sigma
        # is honest. The correction bound is the safety, not the sigma.
        self._grid_sigma_deg = float(
            self.declare_parameter('grid_sigma_deg', 1.0).value)
        self._grid_max_corr_deg = float(
            self.declare_parameter('grid_max_correction_deg', 20.0).value)
        # Off by default. The board's yaw is already a fused 500 Hz solution and
        # feeding it back in as a measurement makes this filter agree with it by
        # construction -- which looks like convergence and measures nothing.
        # Turn it on when the anchor has made heading absolute, not before.
        self._use_yaw = bool(self.declare_parameter('use_yaw', False).value)
        # Velocity from commanded demand, learned against flow. ON by default
        # because it is silent until it has LEARNED: an unready model aids
        # with nothing, so the default cannot inject a guessed gain.
        self._model = CommandVelocityModel(
            tau_s=float(self.declare_parameter('demand_tau_s', 1.0).value))
        self._model_aid = bool(self.declare_parameter('demand_aid', True).value)
        # Apply every measurement at the instant it describes (retro.py). OFF by
        # default until its Pi cost is measured: flow ~0.8 ms per sample 60 ms
        # late and a fix ~19 ms per second late, on the dev box.
        self._retro = None
        if bool(self.declare_parameter('retrodict', False).value):
            self._retro = Retrodictor(
                self._filter,
                horizon_s=float(self.declare_parameter('retro_horizon_s', 2.0).value))
        self._last_demand_t = None
        self._centred_since = None       # when the demand last became centred
        self._motion = MotionCheck()
        self._motion_state = None

        sensor_qos = QoSProfile(depth=20,
                                reliability=QoSReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Imu, '/mongla/imu', self._on_imu, sensor_qos)
        self.create_subscription(MonglaState, '/mongla/state', self._on_state, 10)
        self.create_subscription(
            TwistWithCovarianceStamped,
            f'/mongla/vision/{cam}/velocity', self._on_flow, sensor_qos)
        # The pool-frame fix from `mongla.fix_position()` -- prop resection.
        # This is the only channel that bounds horizontal drift, so it is the
        # one whose absence is worth noticing in the diagnostic line.
        self.create_subscription(
            PointStamped, '/mongla/localization/fix', self._on_fix, 10)
        # The anchored world heading, latched by `anchor_on()`. Until one
        # arrives the filter's attitude is the BOARD's, which is boot-relative
        # or magnetic -- a perfectly good attitude in a frame that is not the
        # pool's. Receiving this is what makes the output frame honest.
        latched = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Float32, '/mongla/localization/heading',
                                 self._on_heading, latched)
        self.create_subscription(Vector3Stamped, '/mongla/demand',
                                 self._on_demand, sensor_qos)
        # The floor's grid, which BOUNDS yaw drift without a magnetometer.
        self.create_subscription(
            FloorHeading, f'/mongla/vision/{cam}/floor_grid_deg',
            self._on_floor_grid, 10)
        # The lane line: the same drift bound with HALF the aliasing (a line
        # is identical from two directions, a grid from four). Shares the
        # grid's sigma and correction bound -- one floor-geometry pair, not two.
        self.create_subscription(
            FloorHeading, f'/mongla/vision/{cam}/lane_heading_deg',
            self._on_lane_line, 10)

        self._pub = self.create_publisher(Odometry, '/mongla/odom', 10)
        # The tree (issue #19). The static edges -- FRD<->FLU, NED<->ENU, the
        # measured camera mounts -- are the manager's, because it always runs;
        # this node owns only what the filter knows: `odom_ned -> mongla`, and
        # `map -> odom` once the pool anchor has been applied.
        from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
        self._tf = TransformBroadcaster(self)
        self._tf_static = StaticTransformBroadcaster(self)
        # 'ok' | 'blocked' | 'unknown', latched and published on change. Read
        # by `mongla.motion()`: a timed move into a prop reports success, and
        # this is the only thing that can tell the mission it went nowhere.
        self._pub_motion = self.create_publisher(String, '/mongla/localization/motion', latched)
        # 'aided' | 'unaided' -- is horizontal position OBSERVABLE right now?
        #
        # Why this is a topic and not a field on `/mongla/state`. That message
        # is `auv_manager_node`'s and describes the BOARD -- armed, mode, yaw,
        # depth, battery. The filter lives in this process, so routing a
        # localization fact through the manager would need a new `.msg` field
        # AND a topic to carry it across the process boundary anyway. The fact
        # belongs on the node that owns it.
        #
        # And it is not `/mongla/localization/motion`. That one answers "is
        # the hull physically moving as commanded" -- a prop, a snag, a dead
        # thruster. This answers "can the filter see where it is". A blocked
        # hull is perfectly well localised; an unaided one is not.
        #
        # `unaided` means B-56's regime: `RIEKF._velocity_is_observed()` is
        # False, so depth and yaw updates are being REFUSED and only a landmark
        # fix can still pin position. The pose keeps publishing and keeps
        # looking healthy, which is exactly why this has to be said out loud.
        self._pub_aiding = self.create_publisher(
            String, '/mongla/localization/aiding', latched)
        self.create_timer(0.1, self._publish)
        self.create_timer(5.0, self._diagnose)
        self.get_logger().info(
            f'[LOCAL] invariant filter up: imu=/mongla/imu '
            f'flow=/mongla/vision/{cam}/velocity use_yaw={self._use_yaw}')

def main(args=None):
    from mongla_localization.orphan import die_with_parent
    die_with_parent()        # an orphan holds devices and duplicates topics
    rclpy.init(args=args)
    node = LocalizationNode()
    from mongla_localization.params_file import refuse_unknown
    refuse_unknown(node)       # bringup.yaml `nodes:` typos stop here
    try:
        from mongla_localization.spin import spin
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
