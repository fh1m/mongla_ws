"""DetectorNode, part 1 of 4: parameters, model loading, and live parameter changes.

A model swap on the day is one `set_model`; everything that swap touches is here.
"""
import os
from typing import List, Optional
from mongla_vision.detection.factory   import make_detector
from .detector_params import _model_stem, _parse_model_conf, _split_active


class DetectorSetupMixin:
    """Methods of `DetectorNode`; see `detector_node.py` for the node."""

    def _declare_parameters(self) -> None:
        """Every ROS parameter this node reads, with its default.

        One place to look for a knob; values are read in __init__.
        """
        self.declare_parameter('camera',              'laptop')
        self.declare_parameter('image_topic',         '')
        self.declare_parameter('model_path',          'yolov11n')
        self.declare_parameter('models',              '')     # CSV name=stem pairs
        self.declare_parameter('active_model',        '')     # registry key to start with
        self.declare_parameter('device',              'auto')
        self.declare_parameter('half',                True)   # fp16: ~half the VRAM; coerced off on non-CUDA (yolo.py)
        self.declare_parameter('conf',                0.35)
        # BYTE association floor: PUBLISH down to this, while `conf` stays the
        # floor the control loop steers on (`vision.ctrl_conf`). 0.0 = OFF, and
        # off is byte-for-byte the previous behaviour.
        #
        # Measured on real footage, publish floor the ONLY variable: presence on
        # hard clips 8.3 -> 34.8 %, and NO change on clips already at 100 %. The
        # low-score boxes are the occluded and motion-blurred ones (ByteTrack),
        # which is exactly the AUV case -- our own underwater score p50 is 0.258
        # against a shipped 0.25 floor, so we were discarding the median.
        self.declare_parameter('assoc_conf',          0.0)
        # Per-model confidence overrides: CSV 'name=conf' (e.g.
        # 'torpedo_blood_hole=0.55,gate_sharks=0.35'). Applies on top of
        # the uniform `conf` above, targeting individual registry entries, and
        # PERSISTS across active_model switches (each detector holds its own
        # threshold). Empty = every model uses `conf`. Live-tunable.
        self.declare_parameter('model_conf',          '')
        self.declare_parameter('iou',                 0.5)
        self.declare_parameter('imgsz',               640)
        self.declare_parameter('max_det',             100)   # post-NMS cap (live-tunable)
        self.declare_parameter('classes',             'person')
        # Outlines are EVIDENCE, and evidence should be switchable at the
        # pool rather than in a branch -- the same argument as `claw_required:
        # false` in the champions' configs. Measured cost at the counts a
        # mission produces: 0.075 ms for one box and 0.46 ms for three masks
        # against a ~10.5 ms frame, so this exists for an operator who wants
        # the topic gone, not because the cost forces a choice.
        self.declare_parameter('publish_contours',    True)
        # Mask decoding, live. Only a segmentation model can honour it; a box
        # detector has no masks and says so rather than accepting silently.
        self.declare_parameter('masks',               True)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('debug_image_hz',      5.0)
        self.declare_parameter('alignment_deadband',  0.05)
        self.declare_parameter('paused',              False)
        # ON BY DEFAULT, WITH A FALLBACK, because "off" and "on" both have a
        # silent failure mode and only one of them is recoverable.
        #
        # Direct feed is the arrangement the vehicle runs: the camera in this
        # process hands frames over by reference. Defaulting it OFF meant the
        # composed launcher had to remember to switch it on, and forgetting
        # left the detector subscribing AND being fed -- the same picture
        # decoded and inferred twice, with the slower copy winning half the
        # time.
        #
        # Defaulting it ON has the opposite risk: a STANDALONE `detector_node`
        # (vision.launch.py, replay) has no camera in its
        # process, so it would sit forever receiving nothing, silently. That
        # is the exact class of bug this round has been removing.
        #
        # So it is on, and it SELF-CORRECTS: if no frame arrives by reference
        # within `_DIRECT_FALLBACK_S`, the node subscribes to the topic and
        # says so loudly. A misconfiguration costs one warning and two
        # seconds, never a dead detector.
        self.declare_parameter('direct_feed',         True)
        # UNDERWATER CONTRAST PREPROCESSING. 'clahe' or 'off'.
        #
        # Measured on real RoboSub 2025 footage: on the gate approach -- 3.7x
        # blurrier than the bin clip at the same brightness -- it takes target
        # presence from 10.7 % to 56.2 % and the mean score from 0.226 to
        # 0.410. On footage that already works it costs nothing: bin stays
        # 100 %, octagon stays 100 % with a HIGHER mean score.
        #
        # Off by default because it is 3.78 ms on the Pi (77 Hz -> ~46), which
        # is a real trade the operator should make deliberately.
        self.declare_parameter('preprocess',          'auto')
        self.declare_parameter('preprocess_clip',     0.0)
        # RANGE CROP. Feed the detector a centre crop while the target is far,
        # so it occupies more of the 640x640 the chip sees. Measured on real
        # labelled data: recall at 4x the training distance goes 65.9 % ->
        # 100 %, and at 6.7x, 20.2 % -> 67.4 %. It releases automatically on
        # approach because a fixed crop LOSES close targets (69 % at 1x).
        # `imgsz` cannot do this on the Hailo -- it is baked into the HEF.
        # An INT, not a bool: 0/1 are the operator's answer and -1 is
        # 'not set', which a bool cannot express -- and without a third
        # state a profile can never turn this on.
        self.declare_parameter('range_crop',          -1)
        # ONE WORD instead of four knobs. `vision:=murky` on competition day
        # beats getting conf/preprocess/clip/crop right under a run clock.
        # A profile supplies DEFAULTS ONLY -- an explicit parameter still
        # wins, so debugging keeps the individual knobs.
        self.declare_parameter('vision_profile',      '')

    # ------------------------------------------------------------------ #
    #  Running more than one model on the same frame
    # ------------------------------------------------------------------ #
    def _set_extra(self, names) -> List[str]:
        """Resolve the SECONDARY models -- the ones run beside the primary.

        ⛔ WHY A SECONDARY LIST AND NOT A PLURAL `self._det`. Everything that
        makes a model "the" model -- the class allowlist, `vision_info`, the
        alignment line, the debug overlay's model tag -- is written against a
        single active detector in eighteen places. Pluralising that would turn
        a wiring change into a rewrite of the node's identity handling, and
        every one of those eighteen would then have to decide what "the model"
        means when there are two. The primary keeps its meaning; the extras
        only contribute detections.

        A key that does not resolve is dropped with a LOUD error rather than
        refused, for the same reason the registry tolerates a model whose
        weights are missing: losing the second model must not cost the run.
        """
        self._extra = []
        self._extra_names = []
        for name in names:
            key = self._resolve_model_key(name)
            if key is None or self._registry.get(key) is None:
                self.get_logger().error(
                    f"[DET  ] active_model names {name!r} as a second model "
                    f"and it is not loaded -- running without it. keys="
                    f"{sorted(self._registry)}")
                continue
            if key == self._active_name:
                continue           # naming the primary twice is not two models
            self._extra.append(self._registry[key])
            self._extra_names.append(key)
        if self._extra_names:
            self.get_logger().warning(
                f"[DET  ] running {1 + len(self._extra_names)} models on every "
                f"frame: {self._active_name!r} + {self._extra_names}. The chip "
                f"runs one graph at a time and each handover costs ~4 ms, so "
                f"the frame rate is the SUM plus the swaps -- measured 37.5 Hz "
                f"per pair against 95 Hz for one. Deliberate, not free.")
        return self._extra_names

    def _merge_extra(self, frame, detections: list) -> list:
        """Run each secondary model on the same frame and merge the results.

        ⛔ CLASS IDS ARE REMINTED FROM LABELS WHEN MODELS ARE MERGED, and only
        then. Two models each number their own classes from zero, so
        `gate`(0) from one and `person`(0) from the other collide in-process --
        the debug palette gives them one colour and any consumer that keys on
        the integer merges two classes. `class_index` is the same label->id
        table the wire round-trip already uses, so the merged ids agree with
        what a subscriber will mint. The single-model path does not go through
        here and is byte-identical to before.
        """
        # `getattr`, not `self._extra`: a node built by a harness that
        # bypasses both init paths has no such attribute, and an AttributeError
        # here would surface as an inference failure rather than as itself.
        if not getattr(self, '_extra', None):
            return detections
        from .detection.messages import remint_class_ids
        out = list(detections)
        for det in self._extra:
            try:
                out.extend(det.infer(frame))
            except Exception as exc:                        # noqa: BLE001
                self.get_logger().warning(
                    f"[DET  ] secondary model failed on this frame: {exc!r}")
        return remint_class_ids(out)

    def _load_single_model_async(self, *, model_path, device, conf, iou, imgsz, half, max_det, allowlist):
        """Background thread: load the detector, then go live. Node subscribes before this runs."""
        # Kept so a recovery can rebuild through THIS path rather than a second
        # copy of the construction that would drift from it.
        self._build_kwargs = dict(model_path=model_path, device=device,
                                  conf=conf, iou=iou, imgsz=imgsz, half=half,
                                  max_det=max_det, allowlist=allowlist)
        try:
            det = make_detector(
                model_path=model_path,
                device=device, conf=conf, iou=iou, imgsz=imgsz,
                half=half, max_det=max_det, class_allowlist=allowlist,
                logger=self.get_logger())
        except Exception as exc:
            # ⛔ D16's TWIN, AT INIT. Returning here left `self._det` None and
            # the infer loop then dropped every frame at `if det is None:
            # continue` -- silently, forever, while the node answered param
            # queries and logged "chip efficiency". Observed on the vehicle:
            # `Failed to open device file /dev/hailo0 with error 6` after a
            # restart race, and the stack looked healthy from every angle
            # except the detection rate.
            #
            # A node that has no model cannot detect anything, so it must not
            # keep claiming to be up. Exit and let a supervisor restart it --
            # which also resolves the common cause, a previous process still
            # holding the device.
            hook = getattr(self, '_init_failure_fn', None)
            if hook is not None:
                # Composed with another camera (detector_dual_node): the owner
                # decides. Killing the process here took a WORKING detector on
                # the other camera down with it (2026-10-03, on the vehicle:
                # forward gate_sharks.hef live, downward model missing its
                # .hef -> both gone).
                self.get_logger().fatal(f"[DET  ] detector init FAILED: {exc}")
                hook(self, exc)
                return
            self.get_logger().fatal(
                f"[DET  ] detector init FAILED: {exc}. EXITING: without a model "
                f"this node would drop every frame silently while looking "
                f"healthy. A supervisor restart also clears a device still "
                f"held by a previous process.")
            os._exit(2)
        # Apply any allowlist change that arrived during load via a param callback.
        pending = self._pending_allowlist
        if pending is not allowlist:
            det.update_allowlist(pending)
        self._apply_assoc_conf(det)
        self._det = det  # atomic publish under CPython GIL — _infer_loop sees it next tick
        # Re-apply a per-model conf override that was set before the model landed
        # (startup or an early live set_conf): _apply_model_conf ran against a None
        # detector then, so replay it now the single model exists.
        if getattr(self, '_pending_model_conf', ''):
            self._apply_model_conf(self._pending_model_conf)
        self.get_logger().info("[DET  ] model ready — inference active")

    def _apply_model_conf(self, value: str) -> None:
        """Apply per-model conf overrides (CSV 'name=conf') to the registry.

        Each named entry gets its own threshold, persisting across active_model
        switches (the override lives on the detector). In single-model mode a
        pair naming the loaded model (or its stem) applies to it. Unknown names
        are warned, not fatal -- a live-tuned param must never crash the node.
        """
        overrides = _parse_model_conf(value)
        if not overrides:
            return
        for name, conf in overrides.items():
            det = None
            if self._registry:
                # Resolve by registry KEY or model STEM, mirroring the
                # active_model handler -- an aliased registry (models:=gate=stem)
                # must accept set_conf(model='<stem>') too, or the per-model conf
                # silently no-ops (the b155736 identity-bug class).
                key = self._resolve_model_key(name)
                det = self._registry[key] if key is not None else None
            elif self._det is not None and (
                    name == '' or _model_stem(name) == self._single_model_name):
                # Single-model launch: _active_name stays None, so match on the
                # loaded model's STEM instead (else a valid set_conf(model='<stem>')
                # silently drops -- the exact torpedo "run this model tight" case).
                det = self._det
            if det is None:
                self.get_logger().warning(
                    f"[DET  ] model_conf: {name!r} not in "
                    f"{sorted(self._registry) if self._registry else 'single-model'}")
                continue
            det.update_conf(conf)
            self.get_logger().info(f"[DET  ] model_conf {name!r} → {conf:.3f}")

    def _apply_assoc_conf(self, det) -> None:
        """Push the BYTE publish floor onto a freshly built detector.

        Called at EVERY construction site (registry, single, async) rather than
        once at startup: a model loaded later would otherwise silently keep the
        default floor, which is the shape of bug this package has shipped three
        times (device_path into **_, the unloaded YAML table, ros2 param set on
        a construction-time param).
        """
        a = float(self.get_parameter('assoc_conf').value)
        if a > 0.0 and hasattr(det, 'update_assoc_conf'):
            det.update_assoc_conf(a)

    def _resolve_model_key(self, name: str) -> Optional[str]:
        """Map a set_model()/active_model argument to a registry key.

        Accepts EITHER the launch registry key (``mongla.use('gate')`` with
        ``models:=gate=...``) OR the model stem (``set_model('gate_sharks')``
        / a ClassRef). Returns the key, or ``None`` if neither matches. Registry
        mode only (single-model handled separately).
        """
        if name in self._registry:
            return name
        return self._stem_to_key.get(_model_stem(name))

    def _on_parameter_change(self, params):
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == 'classes':
                classes_str = str(p.value).strip()
                new_allow   = (
                    None if not classes_str
                    else [c.strip() for c in classes_str.split(',') if c.strip()]
                )
                self._pending_allowlist = new_allow  # persists for async load
                if self._det is not None:
                    self._det.update_allowlist(new_allow)
                self._publish_classes(classes_str)
                self.get_logger().info(
                    f"[DET  ] classes → {new_allow or '*all*'}")

            elif p.name == 'active_model':
                name = str(p.value).strip()
                if not self._registry:
                    # Single-model launch: set_model to the loaded model is a no-op
                    # SUCCESS (so ClassRef/use('<stem>') missions work); any other
                    # name is a clear, actionable reject (no 'registry' word, so the
                    # DSL surfaces THIS message instead of a generic wrap).
                    if (name and self._single_model_name
                            and _model_stem(name) == self._single_model_name):
                        self.get_logger().info(
                            f"[DET  ] active_model {name!r} already loaded "
                            f"(single-model launch) — no-op")
                        continue
                    return SetParametersResult(
                        successful=False,
                        reason=(f"single-model launch loaded "
                                f"{self._single_model_name!r}; cannot switch to "
                                f"{name!r} live -- relaunch with model:={name} (or "
                                f"models:=... for hot switching)"))
                name, extra_names = _split_active(name)
                key = self._resolve_model_key(name)
                if key is None:
                    return SetParametersResult(
                        successful=False,
                        reason=(f"active_model={name!r} not found -- keys="
                                f"{sorted(self._registry)} stems="
                                f"{sorted(self._stem_to_key)}"))
                self._det = self._registry[key]
                self._active_name = key
                self._set_extra(extra_names)
                self._publish_vision_info()
                self.get_logger().info(
                    f"[DET  ] active_model → {key!r}"
                    + (f" (via stem {name!r})" if key != name else "")
                    + (f" + {self._extra_names}" if self._extra_names else ""))

            elif p.name == 'paused':
                state = 'paused' if p.value else 'resumed'
                self.get_logger().info(f"[DET  ] {state}")

            elif p.name == 'conf':
                new_conf = float(p.value)
                # Apply across the whole registry (mirror max_det) so a live
                # set_conf survives a later set_model switch -- otherwise it only
                # touched the active model and reverted to the launch conf on the
                # next switch. Single-model mode just updates self._det.
                for det in (self._registry.values() if self._registry
                            else ([self._det] if self._det is not None else [])):
                    det.update_conf(new_conf)
                self.get_logger().info(f"[DET  ] conf → {new_conf:.3f}")

            elif p.name == 'assoc_conf':
                a = float(p.value)
                for det in (self._registry.values() if self._registry
                            else ([self._det] if self._det is not None else [])):
                    if hasattr(det, 'update_assoc_conf'):
                        det.update_assoc_conf(a if a > 0.0 else 1.0)
                self.get_logger().info(
                    f"[DET  ] assoc_conf → {a:.3f}"
                    + ("  (OFF)" if a <= 0.0 else
                       "  -- boxes below the control floor now reach the tracker "
                       "for ASSOCIATION only"))

            elif p.name == 'model_conf':
                # Per-model override (CSV 'name=conf'); targets individual
                # registry entries and persists across active_model switches.
                self._pending_model_conf = str(p.value)
                self._apply_model_conf(self._pending_model_conf)

            elif p.name == 'masks':
                want = bool(p.value)
                takers = [d for d in (self._registry.values() if self._registry
                                      else ([self._det] if self._det is not None else []))
                          if hasattr(d, 'set_masks')]
                for det in takers:
                    det.set_masks(want)
                if takers:
                    self.get_logger().info(
                        f"[DET  ] masks → {want} ({len(takers)} segmentation "
                        f"model(s))")
                else:
                    # Not an error: a box-only launch is a valid configuration.
                    # But saying nothing would let a mission believe it had
                    # turned outlines on when no loaded model can produce one.
                    self.get_logger().warning(
                        f"[DET  ] masks → {want} IGNORED: no loaded model is a "
                        f"segmentation model. Contours will stay box corners.")

            elif p.name == 'max_det':
                new_max = int(p.value)
                # Apply across the whole registry so a model switch keeps the cap.
                for det in (self._registry.values() if self._registry
                            else ([self._det] if self._det is not None else [])):
                    det.update_max_det(new_max)
                self.get_logger().info(f"[DET  ] max_det → {new_max}")

        return SetParametersResult(successful=True)
