"""VehicleProfile — capability flags for the running AUV body.

Mongla (duburi_ws) runs on two bodies:
  • Duburi 4.5  — has DVL (Nortek Nucleus1000) + grabber/dropper/torpedo
  • Dubomini 2.0 — no DVL, no manipulators

States branch on profile flags so the same plan builder generates the
correct FSM for either vehicle with zero mission-code edits.

Usage:
    profile = VehicleProfile.auto(node)      # probe /duburi_manager at runtime
    profile = VehicleProfile.duburi45()      # hardcoded for Duburi 4.5
    profile = VehicleProfile.dubomini()      # hardcoded for Dubomini 2.0
"""
from __future__ import annotations

import time
from dataclasses import dataclass

_DVL_SOURCES = {'dvl', 'nucleus_dvl', 'bno085_dvl', 'dvl_bno'}

_PROBE_TIMEOUT_S = 3.0


@dataclass(frozen=True)
class VehicleProfile:
    name: str
    has_dvl: bool
    has_manipulators: bool
    mission_depth_m: float  # negative = submerged
    # Which flight controller is driving. This used to be invisible here, and the
    # profile inferred vehicle identity from the YAW-SOURCE STRING alone -- so on
    # the srot backend `yaw_source:=dvl` (the launch default) set has_dvl=True and
    # every MoveForwardState with a distance_m dispatched `move_forward_dist`,
    # which srot_fc hard-refuses. Worse, those call sites are `if dist: ... elif
    # duration: ...`, so the refusal did NOT fall back to the timed leg -- the
    # state just failed.
    flight_controller: str = 'pixhawk'

    @property
    def has_distance_moves(self) -> bool:
        """True when `move_*_dist` will actually run.

        Needs a DVL *and* a backend that implements the streamed distance path.
        srot does not (`srot_fc.UNSUPPORTED_VERBS`) -- the DVL is host-side only
        and never reaches the board. Branch on THIS, not on `has_dvl`, wherever a
        distance verb is about to be issued.
        """
        return self.has_dvl and self.flight_controller != 'srot'

    @property
    def has_heading_lock(self) -> bool:
        """True when `lock_heading` holds anything.

        Refused on srot: the board locks the heading each translate leg starts
        with, on-board, which is what HeadingLock was invented to fake against an
        untrusted ArduSub compass. A plan that needs an explicit lock must branch.
        """
        return self.flight_controller != 'srot'

    # ------------------------------------------------------------------
    # Named constructors
    # ------------------------------------------------------------------

    @classmethod
    def duburi45(cls) -> VehicleProfile:
        return cls(
            name='duburi45',
            has_dvl=True,
            has_manipulators=True,
            mission_depth_m=-0.8,
        )

    @classmethod
    def dubomini(cls) -> VehicleProfile:
        return cls(
            name='dubomini',
            has_dvl=False,
            has_manipulators=False,
            mission_depth_m=-0.6,
        )

    # ------------------------------------------------------------------
    # Runtime probe
    # ------------------------------------------------------------------

    @classmethod
    def auto(cls, node) -> VehicleProfile:
        """Probe /duburi_manager ROS params to detect vehicle capability.

        Falls back to dubomini() profile on any failure — safe default is
        no DVL, no manipulators (prevents unintended actuations).
        """
        try:
            from rcl_interfaces.srv import GetParameters
            import rclpy

            client = node.create_client(GetParameters, '/duburi_manager/get_parameters')
            if not client.wait_for_service(timeout_sec=_PROBE_TIMEOUT_S):
                node.get_logger().warn(
                    '[VehicleProfile.auto] /duburi_manager not reachable — '
                    'falling back to dubomini() profile (safe default)'
                )
                return cls.dubomini()

            req = GetParameters.Request()
            req.names = ['yaw_source', 'dvl_auto_connect', 'flight_controller']
            future = client.call_async(req)

            deadline = time.monotonic() + _PROBE_TIMEOUT_S
            while not future.done():
                rclpy.spin_once(node, timeout_sec=0.1)
                if time.monotonic() > deadline:
                    node.get_logger().warn('[VehicleProfile.auto] param read timed out')
                    return cls.dubomini()

            resp = future.result()
            yaw_src   = resp.values[0].string_value if resp.values[0].type == 4 else 'mavlink_ahrs'
            dvl_auto  = resp.values[1].bool_value   if len(resp.values) > 1 else True
            fc_kind   = (resp.values[2].string_value
                         if len(resp.values) > 2 and resp.values[2].type == 4
                         else 'pixhawk')
            fc_kind   = (fc_kind or 'pixhawk').strip().lower()
            has_dvl   = yaw_src in _DVL_SOURCES and dvl_auto

            profile = cls(
                name='duburi45' if has_dvl else 'dubomini',
                has_dvl=has_dvl,
                has_manipulators=False,  # auto() can't probe manipulators from ROS params;
                                         # payload fire works via PayloadDriver regardless.
                                         # Use VehicleProfile.duburi45() to enable manipulator states.
                mission_depth_m=-0.8 if has_dvl else -0.6,
                flight_controller=fc_kind,
            )
            node.get_logger().info(
                f'[VehicleProfile.auto] detected: {profile.name} '
                f'(dvl={has_dvl}, yaw_source={yaw_src})'
            )
            return profile

        except Exception as exc:
            try:
                node.get_logger().warn(
                    f'[VehicleProfile.auto] probe failed ({exc}) — '
                    'falling back to dubomini() profile'
                )
            except Exception:
                pass
            return cls.dubomini()

    def __str__(self) -> str:
        return (
            f'VehicleProfile({self.name}, dvl={self.has_dvl}, '
            f'manip={self.has_manipulators}, depth={self.mission_depth_m}m)'
        )
