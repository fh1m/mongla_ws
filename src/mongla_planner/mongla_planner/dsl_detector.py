"""Tuning the detectors and the manager mid-mission: models, classes, confidence, parameters.

Mixed into `MonglaMission` (mongla_dsl.py), which owns the client, the
node and the caches these methods read.
"""
from __future__ import annotations

import time as _time

import rclpy
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter

from .dsl_support import _param_value


class DetectorMixin:
    def active_models(self, camera: str | None = None, *, timeout: float = 1.0):
        """`(model_stems, epoch)` the detector says is live on `camera`, or None.

        Read from the detector's latched `vision_info`: `database_location` is the
        comma list of live stems (primary first) and `database_version` an epoch
        bumped on every switch. None when the detector has not announced one.
        """
        import time as _t
        from vision_msgs.msg import VisionInfo
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        cam = camera or self.camera
        subs = self.__dict__.setdefault('_vinfo_subs', {})
        latest = self.__dict__.setdefault('_vinfo', {})
        node = self.client.node
        if cam not in subs:
            def _keep(msg, c=cam):
                stems = tuple(x for x in str(msg.database_location).split(',') if x)
                latest[c] = (stems, int(msg.database_version))
            subs[cam] = node.create_subscription(
                VisionInfo, f'/mongla/vision/{cam}/vision_info', _keep,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL))
        deadline = _t.monotonic() + float(timeout)
        rclpy.spin_once(node, timeout_sec=0.0)
        while cam not in latest and _t.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        return latest.get(cam)

    def _confirm_model(self, camera: str, name: str, before, timeout_s: float) -> None:
        """Block until `vision_info` shows `name` live with a NEW epoch, then drop
        every detection cached before it. Raises if the detector never confirms.

        WHY: the parameter set returning is not the model running. Until the
        detector republishes `vision_info`, frames in the cache came from the OLD
        model, and a `detected()` right after a switch can answer about a class
        the new model does not even have.
        """
        import time as _t
        want = [x.strip() for x in name.split(',') if x.strip()]
        if before and list(before[0][:len(want)]) == want:
            # Already the live selection: the detector does not re-announce a
            # no-op, and nothing cached came from a different model.
            self.log.info(f'[DSL  ] {camera}: model {name!r} was already live')
            return
        # Names, not the epoch: the epoch resets to 1 when a detector restarts,
        # so "newer epoch" would refuse a real switch on a respawned node.
        deadline = _t.monotonic() + timeout_s
        got = None
        while _t.monotonic() < deadline:
            got = self.active_models(camera, timeout=0.05)
            if got and list(got[0][:len(want)]) == want:
                self._det_cache.pop(camera, None)
                self._det_seen.pop(camera, None)
                self.log.info(f'[DSL  ] {camera}: model {name!r} confirmed live '
                              f'(epoch {got[1]}); pre-switch detections dropped')
                return
        raise RuntimeError(f'set_model({name!r}): detector on {camera!r} did not confirm '
                           f'within {timeout_s:.1f} s (last vision_info: {got})')

    def _detector_node(self, camera: str | None = None,
                       node: str | None = None) -> str:
        """Resolve the detector node name for a camera.

        Single naming rule for the whole stack: ``/mongla_detector_<camera>``
        (e.g. ``/mongla_detector_forward``, ``/mongla_detector_downward``),
        matching the names the vision launch files give the detector nodes.
        ``camera`` defaults to the mission's sticky camera. An explicit
        ``node`` always wins (escape hatch for non-standard setups).
        """
        if node:
            return node
        return f'/mongla_detector_{camera or self.camera}'

    def _detector_present(self, camera: str, *, settle: float = 0.0) -> bool:
        """Graph check: is the ``camera`` detector node currently up?

        Reads the discovery graph (``get_node_names``) -- no service wait -- so
        the exclusivity loop can SKIP an absent counterpart instead of paying
        ``_ensure_detector``'s 5 s ``wait_for_service`` on every switch of a
        single-camera run. Best-effort: any error -> treat as absent (skip).

        `settle` exists because an instant graph read answers "absent" for a
        Node that is running. `get_node_names` reports what DDS discovery has
        found SO FAR, and a mission's first query runs a fraction of a second
        after its own node is created -- before the detector has been
        discovered. Measured on the vehicle: `detected('gate')` returned in
        0.61 s with `live_camera=None` while `ros2 param get
        /mongla_detector_forward paused` answered fine from the same shell.
        A caller that must not act on a false "absent" passes `settle` and
        spins until the node appears or the budget runs out. The exclusivity
        loop keeps the instant read: there, a false absent only skips a pause
        it can redo, and the 5 s service wait is the thing being avoided.
        """
        node = self._detector_node(camera)
        want = node.lstrip('/')
        deadline = _time.monotonic() + max(0.0, float(settle))
        while True:
            try:
                names = self.client.node.get_node_names()
            except Exception:   # noqa: BLE001 -- graph read is best-effort
                return False
            if want in names or node in names:
                return True
            if _time.monotonic() >= deadline:
                return False
            rclpy.spin_once(self.client.node, timeout_sec=0.05)

    def _ensure_detector(self, node: str, *, timeout: float = 5.0) -> None:
        """Abort the mission LOUDLY if detector ``node`` is not on the graph.

        Probed at most once per node (cached in ``_detector_ok``). This is the
        single guard that turns "someone forgot to start the vision stack" from
        a silent warning + a mission that idles on err=+inf (the pool-test
        failure) into an immediate, actionable abort. Called from every detector
        param op AND from the vision verbs (align/move) so a mission that
        skips set_model still can't run blind.
        """
        if node in self._detector_ok:
            return
        ros_node = self.client.node
        cli = ros_node.create_client(GetParameters, f'{node}/get_parameters')
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                cam = node.rsplit('mongla_detector_', 1)[-1]
                raise RuntimeError(
                    f"Detector node {node} NOT FOUND after {timeout:.0f}s -- the "
                    f"vision stack is not running, but this mission uses vision. "
                    f"Start it, e.g.:\n"
                    f"  ros2 launch mongla_vision vision.launch.py camera:={cam} "
                    f"model:=<stem> classes:=<csv>\n"
                    f"(aborting loudly so a missing detector can't cost a run)")
        finally:
            ros_node.destroy_client(cli)
        self._detector_ok.add(node)

    def _refuse_duplicate_node(self, node: str) -> None:
        """Raise if `node`'s name is on the graph more than once.

        Best-effort on the graph read itself: a discovery hiccup must not block
        a legitimate write, so an unreadable graph is treated as "one".
        """
        want = node.lstrip('/')
        try:
            names = list(self.client.node.get_node_names())
        except Exception:   # noqa: BLE001 -- graph read is best-effort
            return
        n = sum(1 for x in names if x.lstrip('/') == want)
        if n > 1:
            raise RuntimeError(
                f'{n} nodes named {node!r} are on the graph -- a parameter '
                f'write would land on an arbitrary one. Stop the duplicate '
                f'stack (orphaned launch?) before tuning.')

    def _set_detector_param(self, node: str, name: str, value) -> None:
        """Set one DETECTOR parameter, aborting loudly if the node is absent.

        `_ensure_detector` is the detector-specific half: "someone forgot to
        start the vision stack" must stop the mission rather than warn, because
        every vision verb after it would idle on err=+inf. The wire work is
        shared with every other node -- see `_set_node_param`.
        """
        self._ensure_detector(node)
        self._set_node_param(node, name, value)

    def _set_node_param(self, node: str, name: str, value) -> None:
        """Set one parameter on ANY node, in-process via SetParameters.

        Replaces the old ``subprocess('ros2 param set')`` which spun up a fresh
        CLI node that had to re-discover the target every call (the flaky,
        silent "Node not found" source). Raises on rejection or timeout so a
        failure is loud rather than a swallowed warning: these are the knobs a
        task's behaviour depends on.
        """
        ros_node = self.client.node
        # A duplicate node name makes this write a coin flip. Two processes
        # can hold the same node name -- an orphaned launch, a second stack
        # started by mistake -- and ROS does not arbitrate: the service call
        # lands on whichever answers, which may be the one NOT driving the
        # vehicle. The setting then reads back correctly from one instance and
        # does nothing to the pipeline, which is indistinguishable from a knob
        # that was never wired. Measured on the vehicle: ten
        # `mongla_tracker_forward` nodes on one graph after repeated launches.
        #
        # Refuse instead. A tuning call that might not take is worse than one
        # that says it cannot.
        self._refuse_duplicate_node(node)
        cli = self._param_clients.get(node)
        if cli is None:
            cli = ros_node.create_client(SetParameters, f'{node}/set_parameters')
            self._param_clients[node] = cli
        if not cli.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(
                f'{node}/set_parameters unavailable -- is that node running?')
        req = SetParameters.Request(
            parameters=[Parameter(name=name, value=_param_value(value))])
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(ros_node, fut, timeout_sec=5.0)
        resp = fut.result()
        if resp is None:
            raise RuntimeError(f'set {node}.{name} timed out (no response)')
        res = resp.results[0]
        if not res.successful:
            raise RuntimeError(f'set {node}.{name}={value!r} rejected: {res.reason}')

    def set_model(self, name, *,
                  camera: str | None = None, node: str | None = None,
                  confirm_s: float = 0.0) -> None:
        """Switch the active detector model, or run SEVERAL at once.

        ``name`` is the model **stem** (e.g. ``'gate_sharks'``) or a
        registry key. Works on BOTH launch styles: a single-model launch
        (``model:=<stem>``) accepts ``set_model('<that stem>')`` as a no-op and
        rejects any *other* name; a registry launch (``models:=``) accepts the
        key OR the stem. Targets ``/mongla_detector_<camera>`` (camera defaults to
        the mission's). Raises if the node is absent or the switch is rejected.

        **Pass a list or a comma-separated string to run more than one model on
        every frame** -- a detector and a segmentation model together, say::

            mongla.use('gate_sharks')                  # detection only
            mongla.use('gate_seg')                            # segmentation only
            mongla.use(['gate_sharks', 'gate_seg'])    # both, merged

        The FIRST name stays "the" model: it owns the class filter, the
        published `vision_info`, and the alignment line. The rest only
        contribute detections, which arrive merged in the same
        `Detection2DArray` and outlined in the same `TargetContours`.

        Two models is not free and is not a default. The accelerator runs one
        graph at a time and each handover costs ~4 ms, so the frame rate is the
        sum plus the swaps -- measured 37.5 Hz for a pair against 95 Hz for one
        model. Ask for both when a task needs both, not for the whole mission.

        Why a widened argument rather than a new verb: the node takes ONE
        `active_model` parameter, and one `SetParameters` call changes the whole
        selection atomically. A separate "extra models" verb would need two
        calls, and between them a frame would be detected with a pair nobody
        asked for.
        """
        if isinstance(name, (list, tuple)):
            name = ','.join(str(n).strip() for n in name if str(n).strip())
        node = self._detector_node(camera, node)
        cam = camera or self.camera
        before = self.active_models(cam, timeout=0.0) if confirm_s > 0.0 else None
        try:
            self._set_detector_param(node, 'active_model', str(name))
        except RuntimeError as exc:
            if 'registry' in str(exc).lower():
                raise RuntimeError(
                    f'set_model({name!r}): {exc} -- launch with models:="..." '
                    f'to enable hot model switching') from None
            raise
        self.log.info(f'[DSL  ] {node} active_model → {name!r}')
        if confirm_s > 0.0:
            self._confirm_model(cam, str(name), before, float(confirm_s))

    def use(self, model: str, classes: str | list | None = None, *,
            camera: str | None = None, node: str | None = None) -> None:
        """Switch active detector model and optionally its class filter.

        Parameters
        ----------
        model : str
            Registry key (e.g. ``'gate'``, ``'combined'``).
        classes : str | list | None
            Class filter to apply. ``None`` leaves current filter untouched.
            ``''`` enables all classes.

        Example::

            mongla.use('gate', 'gate')    # switch model and filter together
            mongla.use('combined', '')    # combined model, all classes visible
        """
        self.set_model(model, camera=camera, node=node)
        if classes is not None:
            self.set_classes(classes, camera=camera, node=node)

    def set_classes(self, classes: str | list, *,
                    camera: str | None = None, node: str | None = None) -> None:
        """Switch the detector's class filter without restarting the node.

        Example::

            mongla.set_classes('gate')
            mongla.set_classes(['gate', 'flare'])
            mongla.set_classes('')   # all classes
        """
        node = self._detector_node(camera, node)
        if isinstance(classes, list):
            classes_str = ','.join(str(c).strip() for c in classes)
        else:
            classes_str = str(classes).strip()
        self._set_detector_param(node, 'classes', classes_str)
        self.log.info(f"[DSL  ] {node} classes → {classes_str!r}")

    def set_conf(self, conf: float, *, model: str | None = None,
                 camera: str | None = None, node: str | None = None) -> None:
        """Set the YOLO confidence threshold live (next inference tick).

        ``model=None`` (default) sets the threshold for EVERY model on the
        detector (survives model switches). ``model='<registry name>'`` sets it
        for that ONE model only -- e.g. run the torpedo model tight and the gate
        model loose on the same forward detector::

            mongla.set_conf(0.35)                              # all models
            mongla.set_conf(0.55, model='torpedo_blood_hole')  # torpedo only

        Per-model needs a ``models`` registry (multi-model launch); the override
        persists across ``set_model`` switches (it lives on the model itself).
        """
        node = self._detector_node(camera, node)
        if model is None:
            self._set_detector_param(node, 'conf', float(conf))
            self.log.info(f"[DSL  ] {node} conf → {float(conf):.3f} (all models)")
        else:
            self._set_detector_param(node, 'model_conf', f'{model}={float(conf)}')
            self.log.info(f"[DSL  ] {node} conf[{model!r}] → {float(conf):.3f}")

    def _subsystem_node(self, kind: str, camera: str | None = None) -> str:
        try:
            pattern = self._NODE_SUFFIX[kind]
        except KeyError:
            raise ValueError(
                f'unknown subsystem {kind!r}; known: '
                f'{sorted(self._NODE_SUFFIX)}') from None
        cam = str(camera or self.camera).strip().lower()
        return '/' + pattern.format(cam=cam)

    def set_node(self, kind: str, *, camera: str | None = None, **params) -> None:
        """Set parameters on ANY node in the stack, by subsystem name.

        Why this exists. A census of the tree found 164 declared parameters
        across 13 nodes and a DSL that could write two groups of them: the
        detector, and the manager's ``vision.*``. Everything else -- the
        tracker's coast and Kalman noise, the camera's exposure and rate, the
        lock ladder's anchor and authority windows, the flow front end's 31
        knobs, PnP's reprojection gate -- was launch-only, which means frozen at
        whatever someone typed before the vehicle went in the water. A
        capability the mission cannot reach is a capability the mission does not
        have.

        One method, seven subsystems, no new layer::

            mongla.set_node('tracker', coast_s=1.2)      # hold a flickering lock
            mongla.set_node('camera',  exposure_us=4000) # kill motion blur
            mongla.set_node('lock',    full_authority_s=3.0)
            mongla.set_node('pnp',     max_reproj_px=15.0)
            mongla.set_node('flow',    max_baseline_s=0.4)
            mongla.set_node('manager', **{'vision.kp_yaw': 70.0})

        ``camera`` selects which per-camera instance; ``flow`` and ``manager``
        are single nodes and ignore it. Raises on a rejected or undeclared
        parameter, for the same reason ``set_detector`` does: a silent no-op is
        a mission believing it changed something.
        """
        if not params:
            raise ValueError('set_node() needs at least one parameter')
        node = self._subsystem_node(kind, camera)
        for name, value in params.items():
            self._set_node_param(node, name, value)
            self.log.info(f'[DSL  ] {node} {name} → {value!r}')

    def set_detector(self, *, camera: str | None = None,
                     node: str | None = None, **params) -> None:
        """Set ANY detector parameter live. The escape hatch, not a shortcut.

        ``set_model`` / ``set_conf`` / ``set_classes`` exist because they do
        more than write a value -- they resolve a registry name, scope a
        threshold to one model, validate a class against the loaded allowlist.
        Everything else the detector node declares is reachable here without a
        new DSL method per knob, so a parameter added to the node is usable
        from a mission the same day::

            mongla.set_detector(masks=False)             # boxes only on transit
            mongla.set_detector(masks=True)              # outlines at the board
            mongla.set_detector(publish_contours=False)  # silence the topic
            mongla.set_detector(max_det=10, iou=0.5)     # several at once

        Order within one call is not guaranteed; make two calls if one setting
        must land before another. Raises on a rejected or undeclared parameter
        -- a silent no-op here would be a mission believing it had changed
        something. That is deliberately the opposite of best-effort: these are
        the knobs a task's behaviour depends on.
        """
        if not params:
            raise ValueError('set_detector() needs at least one parameter')
        target = self._detector_node(camera, node)
        for name, value in params.items():
            self._set_detector_param(target, name, value)
            self.log.info(f'[DSL  ] {target} {name} → {value!r}')

    def lock_class(self, target: str = '', *, camera: str | None = None,
                   timeout: float = 2.0) -> bool:
        """Aim the continuity ladder MID-MISSION. Returns True if it took.

        The ladder (`lock_node`: follower + XFeat anchor) normally aims itself
        at whatever `set_classes` last told the detector, via the latched
        `classes_filter`. Use this only to PIN it somewhere else -- e.g. hold
        the gate while the detector is already hunting the next prop::

            mongla.lock_class('gate')        # pin
            mongla.lock_class('')            # release; follow the mission again

        Unlike the detector helpers this does NOT abort when the node is
        missing. The ladder is a fallback rung: a mission must run without it,
        and a hard failure here would turn an enhancement into a dependency.
        A clear warning is logged instead, and False returned.
        """
        node = f'/mongla_lock_{camera or self.camera}'
        ros_node = self.client.node
        cli = ros_node.create_client(SetParameters, f'{node}/set_parameters')
        try:
            if not cli.wait_for_service(timeout_sec=timeout):
                self.log.warning(
                    f'[DSL  ] {node} not on the graph -- the ladder is not '
                    f'running, so it cannot be aimed at {target!r}. The '
                    f'mission continues on live detections. Start it with '
                    f'`lock:=true` on the vision launch.')
                return False
            from rcl_interfaces.msg import Parameter as _P, ParameterValue as _PV
            from rcl_interfaces.msg import ParameterType as _PT
            req = SetParameters.Request(parameters=[_P(
                name='target_class',
                value=_PV(type=_PT.PARAMETER_STRING,
                          string_value=str(target).strip()))])
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(ros_node, fut, timeout_sec=timeout)
            res = fut.result()
            ok = bool(res and res.results and res.results[0].successful)
            if ok:
                self.log.info(
                    f"[DSL  ] {node} target_class -> {target!r}"
                    + ('' if target else '  (released; follows the mission)'))
            else:
                self.log.warning(f'[DSL  ] {node} refused target_class {target!r}')
            return ok
        finally:
            ros_node.destroy_client(cli)

    def set_vision_param(self, name: str, value: float) -> None:
        """Set a manager ``vision.*`` tunable live (applies to the NEXT vision goal).

        For per-MISSION control of a deck tunable without touching every verb
        call -- e.g. a run in turbid water raises the loss grace ONCE at the top::

            mongla.set_vision_param('lost_grace_s', 1.5)

        ``name`` is the stem with or without the ``vision.`` prefix. The manager
        re-snapshots ``vision.*`` per goal, so the next align picks it up. Raises if
        the manager is absent or the set is rejected (loud, not silent).
        """
        param = name if name.startswith('vision.') else f'vision.{name}'
        self._set_manager_param(param, float(value))
        self.log.info(f'[DSL  ] {self._MANAGER_NODE} {param} → {float(value):.3f}')

    def _set_manager_param(self, name: str, value) -> None:
        """Set one parameter on the manager node via SetParameters (in-process)."""
        node_name = self._MANAGER_NODE
        ros_node = self.client.node
        cli = self._param_clients.get(node_name)
        if cli is None:
            cli = ros_node.create_client(SetParameters, f'{node_name}/set_parameters')
            self._param_clients[node_name] = cli
        if not cli.wait_for_service(timeout_sec=3.0):
            raise RuntimeError(f'{node_name}/set_parameters unavailable')
        req = SetParameters.Request(
            parameters=[Parameter(name=name, value=_param_value(value))])
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(ros_node, fut, timeout_sec=5.0)
        resp = fut.result()
        if resp is None:
            raise RuntimeError(f'set {node_name}.{name} timed out (no response)')
        res = resp.results[0]
        if not res.successful:
            raise RuntimeError(f'set {node_name}.{name}={value!r} rejected: {res.reason}')

    def pause_detector(self, camera: str | None = None, *,
                       node: str | None = None) -> None:
        """Pause inference on a detector node (frame still consumed from queue)."""
        cam  = camera or self.camera
        node = self._detector_node(camera, node)
        self._set_detector_param(node, 'paused', True)
        # Keep the exclusivity tracker honest: if we just paused the live detector,
        # nothing is live now, so the next _activate_camera re-resumes it.
        if self._live_camera == cam:
            self._live_camera = None
        self.log.info(f"[DSL  ] {node} paused")

    def resume_detector(self, camera: str | None = None, *,
                        node: str | None = None) -> None:
        """Resume inference on a detector node."""
        cam  = camera or self.camera
        node = self._detector_node(camera, node)
        self._set_detector_param(node, 'paused', False)
        # This camera is now the live detector. Recording it means a later switch
        # to a DIFFERENT camera pauses THIS one (never two detectors on the Hailo),
        # even when a mission resumed it manually rather than via _activate_camera.
        self._live_camera = cam
        self.log.info(f"[DSL  ] {node} resumed")
