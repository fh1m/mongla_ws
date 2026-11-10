"""LockNode's setup: parameters, the anchor and the bank, and live retuning.
"""
import os

from mongla_vision.anchor import loop_closure as _lc
from mongla_vision.tracking.lock_state import FULL_AUTHORITY_S, ZERO_AUTHORITY_S


class LockSetupMixin:
    """Methods of `LockNode`; see `lock_node.py` for the node."""

    def _declare_parameters(self) -> None:
        """The ROS parameters this node reads, with their defaults.

        Three more are declared where they are read, beside the reasoning about
        them: `target_width_m`, `medium` and `direct_feed` (in lock_wiring.py).
        """
        self.declare_parameter('camera', 'forward')
        self.declare_parameter('target_class', '')
        self.declare_parameter('follow', True)
        self.declare_parameter('anchor', False)
        self.declare_parameter('anchor_model', '')
        # DEFAULT ON since the InferModel port (B-62). Measured on the
        # vehicle: 10.89 ms alone and 19.85 ms with the detector inferring,
        # against 32.9 ms for the ONNX path it replaces -- 1.7x even fully
        # contended, and off the cores the camera pump needs. The two share
        # the chip through HailoRT's scheduler (`detection/hailo.py`); in the
        # composed graph, 17.65 ms p50 in-graph over 378 calls on archive
        # footage (tools/studies/composed_lock_harness.py).
        #
        # Cross-backend equivalence checked before flipping this: 875/888/878
        # of 1024 keypoints match between the ONNX and HEF descriptors, so a
        # checkpoint enrolled on one path is matchable on the other and the
        # bank does not partition by transport. The identity bar is 40.
        #
        # Set false to force the CPU path -- on a dev box with no chip the
        # resolver falls back on its own, so this is for A/B measurement.
        #
        # DEFAULT OFF since 2026-10-03. Chip sharing works only INSIDE one
        # process, and a standalone lock_node runs as its own: it opened a
        # second VDevice, raced the detector process for the chip, and when
        # it won the vehicle had NO detection (HAILO_OUT_OF_PHYSICAL_DEVICES).
        # The numbers above stand; they need the lock composed with the
        # detector to be had safely.
        self.declare_parameter('anchor_xfeat_hef', False)
        # Semi-dense matching: 2.1-2.6x the inliers for +37 % match cost
        # (measured, four frame pairs). DEFAULT OFF until frame-to-reference
        # behaviour and inlier CORRECTNESS are measured -- see
        # never-lose-the-target.md. Hailo backend only; the ONNX path keeps
        # its sparse maths.
        self.declare_parameter('anchor_semi_dense', False)
        # A bank built BEFORE the run, from practice footage or stills:
        # `tools/pool/build_practice_bank.py`. Measured 2026-09-24 -- references
        # from one run clear the trust bar on 92-100 % of frames of a DIFFERENT
        # run of the same prop, so the anchor need not wait for a confident
        # live detection to have something to match against. Which matters
        # because the moment it is needed is the moment the detector is failing.
        #
        # A preloaded reference is trusted exactly like a live one: it clears
        # MIN_INLIERS or it does not answer, and `recognise()` still wants the
        # detector to agree before any identity is asserted.
        self.declare_parameter('anchor_bank', '')
        # Loop closure is off by default, like every other capability here.
        # It is the only path in this node that can reach the LOCALISATION
        # FILTER, and a wrong position fix is not merely wrong -- the filter
        # shrinks its covariance around it and becomes confident. Off until a
        # pool day says otherwise.
        self.declare_parameter('loop_closure',
                               _lc.defaults()['enabled'])
        self.declare_parameter('place_period_s',
                               _lc.defaults()['place_period_s'])
        self.declare_parameter('place_travel_m',
                               _lc.defaults()['place_travel_m'])
        # Height above the floor = pool_depth_m - |depth|. Without it pixels
        # cannot become metres and every closure is refused, by design.
        self.declare_parameter('pool_depth_m',
                               _lc.defaults()['pool_depth_m'])
        # 3 Hz, not 8. Measured on the Pi with the full stack live, and the
        # cost is LATENCY rather than throughput -- detection rate is unchanged
        # at every setting, but the anchor's 33 ms bursts delay the image
        # pipeline:
        #
        #     ladder OFF          det 49.3 Hz   frame age 16.1 ms   idle 82.4 %
        #     follower only       det 50.6 Hz             18.7 ms        75.3 %
        #     + anchor @ 8 Hz     det 50.7 Hz             22.8 ms        65.5 %
        #     + anchor @ 3 Hz     det 50.6 Hz             18.4 ms        66.8 %
        #
        # 8 Hz costs +42 % of frame age for NO extra coverage: the anchor is the
        # long-horizon rung, called on when the follower has already given up,
        # and the p99 gap it exists to cover is 2.418 s. 3 Hz samples that
        # seven times over.
        # Publish rate -- off by default, and the default is the point.
        #
        # The ladder tracks on every camera frame (LK wants the smallest
        # inter-frame motion) and publishes on every frame: MEASURED 132.4 Hz
        # on the vehicle against detections at 30.5 Hz.
        #
        # That is not waste, and an earlier version of this comment said it
        # was. The reasoning "nobody samples above 20-50 Hz" describes the
        # HOST loop, which is a limitation of the un-ported vision uplink --
        # not the architecture. THE CONTROL LOOP IS 500 Hz ON THE SROT BOARD.
        # Once LANDING_TARGET ingest lands the board closes the visual loop at
        # that rate and wants bearings as fresh and as often as we can send
        # them, so capping this publisher would throttle the exact stream the
        # uplink exists to feed.
        #
        # The knob stays for a CPU-bound bench (the ladder costs 16 points of
        # idle: 53.4 % -> 37.4 %), but 0 = publish every frame is the default
        # and the design intent. A RUNG CHANGE is never delayed even when a
        # limit is set -- that transition is the information.
        # The acting bar (B-59). Below this a detection may be ASSOCIATED by
        # the tracker but must not be ACTED ON by the ladder. Default 0.45,
        # deliberately NOT the 0.60 knee that measurement favours: 0.60 is
        # derived from ONE model, ONE class and ONE venue, and shipping a
        # control bar that tight on that evidence would trade a measured false
        # positive for an unmeasured missed detection. 0.45 already removes
        # the 90.8 %-on-empty-water regime (it falls to 46.8 %) while holding
        # 98.9 % on the real gate. Raise it to 0.60 per mission once the sweep
        # has been repeated with visually-confirmed negatives on more venues.
        self.declare_parameter('act_conf', 0.45)
        self.declare_parameter('publish_hz', 0.0)
        # 4.5 Hz, RAISED FROM 3.0 ON A RE-MEASUREMENT (B-62).
        #
        # The 3 Hz below was set when one anchor evaluation was a "33 ms
        # burst" on the CPU. Re-measured on the vehicle after XFeat moved to
        # the accelerator, one full evaluation (detect + match against one
        # stored checkpoint):
        #
        #     CPU   67.0 ms  p95 70.4   -> 3.0 Hz at 20 % duty
        #     HAILO 44.2 ms  p95 46.3   -> 4.5 Hz at 20 % duty
        #
        # The CPU number reproduces the shipped 3.0 Hz exactly from first
        # principles, which is what makes the new one trustworthy: the same
        # arithmetic on the same duty budget, with only the burst changed.
        #
        # What it buys: the anchor pose is REUSED between evaluations, so the
        # box the ladder steers on can be a full period old. 333 ms -> 222 ms
        # of worst-case staleness on the rung that exists to cover the p99
        # 2.418 s gap.
        #
        # The 8 Hz row in the old table (+42 % frame age for no extra
        # coverage) was measured at the OLD burst; 4.5 Hz at 44.2 ms is a
        # LOWER duty than 3 Hz at 67.0 ms, so this costs less latency than
        # what ships today, not more.
        self.declare_parameter('anchor_hz', 4.5)
        self.declare_parameter('full_authority_s', FULL_AUTHORITY_S)
        self.declare_parameter('zero_authority_s', ZERO_AUTHORITY_S)


    def _report_geometry_problems(self):
        """Say what the target table IGNORED, before saying what it used.

        A dropped override and an override that was never read look identical
        from the deck, and a table that failed to parse reads as 'no committed
        width' for every class at once -- which is exactly how a stray indent
        in the committed YAML presented while this was being written.

        Out of line on purpose: `test_target_geometry` asserts the width lookup
        and its assignment stay within one window of source, and inlining this
        pushed them apart.
        """
        from mongla_vision.target_geometry import load_errors, rejected_overrides
        for path, err in load_errors():
            self.get_logger().error(
                f'[LOCK ] target geometry {path} FAILED TO PARSE ({err}) -- '
                f'widths from it are MISSING, not zero')
        for name, w_bad, origin in rejected_overrides():
            self.get_logger().warn(
                f'[LOCK ] IGNORED override {name}={w_bad} m from {origin}: '
                f'outside the plausible band, likely cm typed as m')

    def _build_anchor(self):
        """Optional by design: a missing ONNX must cost the anchor rung, not
        the node -- the follower is still worth running without it."""
        try:
            from mongla_vision.anchor.bank import CheckpointBank
            from mongla_vision.anchor.xfeat_onnx import XFeatONNX
            import glob
            p = str(self.get_parameter('anchor_model').value).strip()
            if not p:
                # MONGLA_HEF_DIR FIRST, then the historical default. The
                # detector already resolves its models that way
                # (`detection/yolo.py`), and hardcoding one of the two paths
                # here made the model directory a truth with two copies: point
                # MONGLA_HEF_DIR somewhere else and the detector follows while
                # the anchor silently does not, losing the rung with one WARN
                # and no error.
                roots = [d for d in (os.environ.get('MONGLA_HEF_DIR', '').strip(),
                                     '~/hailo_models') if d]
                # `.hef` FIRST (B-62). This globbed `xfeat_*.onnx` only, so
                # the compiled HEF sitting in the SAME directory was never a
                # candidate and the anchor ran on the Pi's CPU: measured
                # 32.9 ms per call (30.4 FPS) against 1.43 ms (701 FPS) on the
                # accelerator. A 23x gap, spent on the cores the camera pump
                # and the rclpy executor need.
                #
                # The ONNX stays as the fallback, and it is a real one: the
                # dev box has no chip, and a vehicle whose HEF is missing must
                # still get an anchor rung rather than lose it to a
                # FileNotFoundError.
                # `.hef` FIRST (B-62). It took three attempts to earn this
                # default: an own VDevice gave HAILO_OUT_OF_PHYSICAL_DEVICES,
                # a shared VDevice with a lock gave "Cant activate network
                # because a network is already activated"; the device now runs
                # HailoRT's scheduler, which owns activation, so the anchor and
                # the detectors simply coexist. The ONNX remains the fallback
                # for a box with no chip.
                exts = (('*.hef', '*.onnx')
                        if bool(self._ladder('anchor_xfeat_hef'))
                        else ('*.onnx', '*.hef'))
                c = []
                for r in roots:
                    for ext in exts:
                        c = sorted(glob.glob(os.path.join(
                            os.path.expanduser(r), 'xfeat_' + ext)))
                        if c:
                            break
                    if c:
                        break
                p = c[0] if c else ''
            if not p:
                raise FileNotFoundError('no xfeat_*.hef or xfeat_*.onnx found')
            # `period_s` is what lets the bank size its own shortlist: it
            # measures one match on THIS machine and spends a fixed fraction of
            # the evaluation period. The dev box and the Pi differ by 2.2x on
            # the identical work (14.4 vs 31.4 ms), so a constant width would
            # be wrong on one of them.
            # The accelerator when the artifact is one (B-62), the CPU
            # otherwise. Both backends share `xfeat_onnx`'s post-processing and
            # matcher, so a checkpoint enrolled through either is matchable by
            # the other -- the bank must not partition by transport.
            #
            # Degrades rather than dies. A HEF that will not configure --
            # the detector already holding the device, a driver mismatch, a
            # dev box with no chip at all -- falls back to ONNX with a loud
            # warning. Losing 31 ms per call is bad; losing the anchor rung
            # entirely is worse, and this is the rung that survives a
            # detection gap.
            backend = None
            if p.endswith('.hef'):
                try:
                    from mongla_vision.anchor.xfeat_hailo import XFeatHailo
                    backend = XFeatHailo(
                        p, top_k=1024,
                        semi_dense=bool(self._ladder('anchor_semi_dense')))
                    self.get_logger().info(
                        f'[LOCK ] anchor on the Hailo-8: {os.path.basename(p)}')
                except Exception as exc:                         # noqa: BLE001
                    self.get_logger().warn(
                        f'[LOCK ] {os.path.basename(p)} would not load on the '
                        f'accelerator ({type(exc).__name__}: {exc}) -- falling '
                        f'back to ONNX on the CPU, which measured 32.9 ms per '
                        f'call against 1.43 ms on the chip (B-62).')
                    alt = sorted(glob.glob(os.path.join(
                        os.path.dirname(p), 'xfeat_*.onnx')))
                    p = alt[0] if alt else ''
                    if not p:
                        raise
            if backend is None:
                backend = XFeatONNX(p, top_k=1024)
            self._anchor = CheckpointBank(backend,
                                          period_s=self._anchor_period)
            self.get_logger().info(
                f'[LOCK ] anchor backend {os.path.basename(p)}, '
                f'bank capacity {self._anchor._cap}, '
                f'shortlist <= {self._anchor.shortlist_k()}')
            self._load_bank()
        except Exception as exc:
            self.get_logger().warning(
                f'[LOCK ] anchor DISABLED: {type(exc).__name__}: {exc} '
                f'-- the follower rung still runs')

    def _load_bank(self):
        """Preload a practice bank, if one was named.

        A missing or unreadable bank costs the PRELOAD, never the rung: the
        anchor still learns live checkpoints from detections. But it WARNs
        rather than logging quietly, because an operator who passed
        `anchor_bank:=` and silently got nothing would believe the vehicle had
        a memory it does not have, and would find out during the run.
        """
        path = str(self.get_parameter('anchor_bank').value or '').strip()
        if not path:
            return
        path = os.path.expanduser(path)
        try:
            n = self._anchor.load(path)
            self.get_logger().info(
                f'[LOCK ] preloaded {n} checkpoints from '
                f'{os.path.basename(path)}: {sorted(set(self._anchor.labels))}')
            if self._anchor.labels:
                self._anchor_label = self._anchor.labels[-1]
        except Exception as exc:
            self.get_logger().warn(
                f'[LOCK ] anchor_bank {path} NOT loaded: '
                f'{type(exc).__name__}: {exc} -- the rung still runs and will '
                f'learn checkpoints from live detections')

    def _on_param_change(self, params):
        """Aim (or un-aim) the ladder while a mission is running.

        `target_class` non-empty PINS the aim, exactly as the launch argument
        does. Setting it back to '' releases the pin and the ladder resumes
        following `classes_filter` -- so a mission can take manual control of
        one leg and hand it back, without a relaunch.
        """
        from rcl_interfaces.msg import SetParametersResult
        for prm in params:
            if prm.name == 'target_class':
                new = str(prm.value or '').strip()
                if new == self._cls_pinned:
                    continue
                self._cls_pinned = new
                if new:
                    self._switch_target(new)
                    self.get_logger().info(
                        f'[LOCK ] target_class pinned to {new!r}')
                else:
                    self.get_logger().info(
                        '[LOCK ] target_class released -- following '
                        'classes_filter again')
            elif prm.name == 'publish_hz':
                hz = float(prm.value or 0.0)
                if hz < 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='publish_hz must be >= 0 (0 = no limit)')
                self._pub_min_dt = (1.0 / hz) if hz > 0.0 else 0.0
                self.get_logger().info(
                    f'[LOCK ] publish_hz {"unlimited" if hz <= 0 else hz}')
        return SetParametersResult(successful=True)

    def _on_classes_filter(self, msg):
        """Follow the mission's current target class.

        Only when the filter names EXACTLY ONE class. An empty or multi-class
        filter is not an aim -- steering the ladder at one of several would be
        a guess, and the ladder's whole value is that it does not guess.
        """
        if self._cls_pinned:
            return                        # operator pinned it; do not fight
        names = [n.strip() for n in str(msg.data).split(',') if n.strip()]
        if len(names) != 1 or names[0] == self._cls:
            return
        self._switch_target(names[0])
        self.get_logger().info(
            f'[LOCK ] following {self._cls!r} (from classes_filter)')

    def _switch_target(self, new: str) -> None:
        """Aim at another class, and forget everything about the old one (#75).

        Changing `_cls` alone left the old detection box, the follower's
        points, the anchor's pose and label and the world memory in place,
        so until a detection of the new class arrived the ladder kept
        publishing the OLD target, now read against the new class's width.
        The anchor's bank keeps its references: they are labelled by class,
        and the search is restricted to the new one.
        """
        with self._lock:
            self._gen += 1
            self._cls = new
            self._forget_target_locked()
        self._retarget_width()

    def _forget_target_locked(self) -> None:
        """Drop all per-target state. Call with `self._lock` held."""
        self._det_box, self._det_conf, self._det_cls = None, 0.0, ''
        self._det_t, self._det_header = 0.0, None
        self._held_box, self._held_conf, self._held_t = None, 0.0, 0.0
        self._held_header, self._lock_centre = None, None
        self._confirms = 0
        if self._follower is not None:
            self._follower.drop()
        self._anchor_pose, self._anchor_header, self._anchor_label = None, None, ''
        self._anchor_inliers, self._anchor_best = 0, None
        if self._world is not None:
            self._world = type(self._world)()

    def _retarget_width(self):
        """Re-resolve the committed width for the class we now follow.

        Without this the 6-DoF pose keeps the FIRST class's width and reports a
        confident range for the wrong object -- worse than refusing. An
        explicit `target_width_m` still wins, exactly as at startup.
        """
        if float(self.get_parameter('target_width_m').value or 0.0) > 0.0:
            return
        try:
            from mongla_vision.target_geometry import width_for
        except ImportError:
            return
        w = width_for(self._cls) if self._cls else 0.0
        self._target_w_m = float(w or 0.0)
        if self._target_w_m <= 0.0:
            self.get_logger().warn(
                f'[LOCK ] no committed width for {self._cls!r} -- the 6-DoF '
                f'pose will refuse while this class is the target.')
