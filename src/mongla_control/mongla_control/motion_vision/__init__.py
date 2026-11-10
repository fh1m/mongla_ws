"""Vision-driven closed-loop motion -- two loops, pixel-native.

This module is the control engine behind the two mission verbs
``vision_align`` and ``vision_move``. Everything here is P-control on
**pixel error**, with ``gain`` acting as a hard max-speed cap. There are
no frame-age knobs, no on_lost policies, no lock modes -- the mission DSL
owns search/recovery via a Python ``fallback`` function.

Two loops
---------
``align_loop`` -- centre the target on the selected axes (lat / yaw, and
    on the downward camera image-Y as surge), each with an optional pixel
    offset. Exits when every active
    axis is within ``err_px`` for ``align_stable_frames`` ticks.

``move_loop`` -- drive forward until the target's bounding box fills the
    frame to ``fwd_fill`` (measured by ``mode`` = area/width/height),
    optionally holding a lateral pixel offset (``maintain``). Never
    re-centres yaw; the board holds depth.

Both return an :class:`Outcome` whose ``code`` the facade copies into
``Move.Result.final_value`` so the DSL can branch without the action
client ever raising on a miss.

Sign rules (forward camera, image y grows downward)
---------------------------------------------------
  ex > 0  -> target RIGHT of centre
  ey > 0  -> target BELOW centre
  yaw : target-right -> yaw RIGHT toward it -> NO negate (same polarity as
        lateral; pool-verified -- the old `-ex` negation drove away from the
        target)
  lat : positive = strafe RIGHT, so target-right needs a positive demand -> NO negate

Offsets (pixels, signed): a positive ``lat``/``yaw`` offset keeps the
target that many px to the RIGHT of centre; a positive ``depth`` offset
keeps it that many px BELOW centre. ``0`` = dead centre.
"""


# The loops are split by concern -- control.py (shared), align.py, move.py --
# and re-exported here by name, so `from mongla_control.motion_vision import X`
# keeps working for every X that existed before the split.
from .control import (  # noqa: F401
    ABORTED,
    ALIGNED,
    COMMIT_UNVERIFIED,
    DRIFTED,
    ESTIMATED_CROSSING,
    FWD_BAND,
    KP_FORWARD_DEFAULT,
    KP_LAT_DEFAULT,
    KP_YAW_DEFAULT,
    LOST,
    MIN_ALIGN_ERR_PX,
    NO_CAMERA,
    Outcome,
    SETTLE_PX_DEFAULT,
    TIMEOUT,
    VALID_AXES,
    VALID_MODES,
    VERIFY_TIMEOUT_S,
    VISION_BRAKE_CAP_PCT,
    VISION_BRAKE_GAIN,
    VISION_BRAKE_MIN_PCT,
    VISION_I_LAT_MAX,
    VISION_LOCK_GATE_HZ,
    VISION_LOCK_GATE_MIN,
    VISION_LOCK_GATE_NORM,
    VISION_RANGE_GAIN_FILL_HI,
    VISION_RANGE_GAIN_FILL_LO,
    VISION_YAW_FLOOR_FILL,
    _ALIGN_STABLE_MIN_S,
    _BRAKE_EMA_TAU_S,
    _CODE_NAME,
    _FRAME_EPS_S,
    _PASSTHROUGH_COMMIT_S,
    _STALE_LIMIT_S,
    _VisionLoop,
    _axis_ctrl,
    _brake_axis,
    _camera_ready,
    _centring_error_px,
    _clamp,
    _ema_step,
    _fill,
    _forward_range_hold,
    _live_classes,
    _lock_gate,
    _present,
    _range_gain,
    _tick,
    _tool_offset,
    _verify_after_stop,
    _yaw_command,
)
from .freshness import (  # noqa: F401
    DEROTATE_MAX_AGE_S,
    VISION_FRESH_FULL_MAX_S,
    VISION_FRESH_FULL_S,
    VISION_FRESH_ZERO_MAX_S,
    VISION_FRESH_ZERO_S,
    _FRESH_FULL_K,
    _FRESH_PIPE_K,
    _FRESH_WARN_FLOOR,
    _FRESH_ZERO_K,
    _authority,
    _coast_authority,
    _derotated_ex,
    _fresh_bounds,
    _freshness,
    _warn_low_fps,
)
from .srot_drive import (  # noqa: F401
    SROT_YAW_MIN_PCT,
    _mixer_saturated,
    _srot_drive,
    _srot_yaw_expressible,
)
from .align import (  # noqa: F401
    _Align,
    _Command,
    align_loop,
)
from .move import (  # noqa: F401
    _Move,
    move_loop,
)
from ..motion_rates import VISION_LOOP_HZ as LOOP_HZ  # noqa: F401,E402
