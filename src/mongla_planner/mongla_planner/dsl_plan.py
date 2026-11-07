"""How a mission is budgeted: tasks with deadlines, the run budget, and run_plan.

Mixed into `MonglaMission`.
"""
from __future__ import annotations
import contextlib
from .client import MoveFailed, MoveRejected, TaskAbandoned  # arm() raises MoveFailed; task() raises TaskAbandoned


class PlanMixin:
    """Methods of `MonglaMission`; see `mongla_dsl.py` for the node."""

    @contextlib.contextmanager
    def task(self, name: str, *, deadline_s: float | None = None):
        """Bound a block of verbs by time, and abandon it cleanly when it runs out.

        OPT-IN. When the deadline passes, the goal in flight is CANCELLED (the
        manager brakes and neutralises as for any cancel) and `TaskAbandoned`
        is raised out of the block, so the mission takes its fallback::

            try:
                with mongla.task('torpedo', deadline_s=90):
                    mongla.vision.align('hole', ...)
            except TaskAbandoned:
                mongla.fire(1)                      # blind shot, keep the points

        With no `deadline_s` and a budget on (`use_budget`), the deadline is what
        the budget has left. With neither, the block only records its outcome.
        Nested tasks keep the EARLIER deadline. `stop`/`surface`/`disarm`/`pause`
        are never blocked by a passed deadline.
        """
        import time as _t
        from mongla_planner.client import TaskAbandoned
        budget = getattr(self, '_budget', None)
        if deadline_s is None and budget is not None and budget.started:
            deadline_s = budget.remaining_s()
        start = _t.monotonic()
        outer = getattr(self.client, '_task_deadline', None)
        mine = start + float(deadline_s) if deadline_s is not None else None
        eff = mine if outer is None else (outer if mine is None else min(outer, mine))
        self.client._task_deadline = eff
        outcome = 'done'
        try:
            yield
        except TaskAbandoned:
            outcome = 'abandoned'
            self.log.warning(f'[TASK ] {name}: ABANDONED at deadline '
                             f'({_t.monotonic() - start:.1f} s)')
            raise
        except Exception as exc:
            outcome = f'failed: {exc}'
            raise
        finally:
            self.client._task_deadline = outer
            self.__dict__.setdefault('_scoreboard', []).append({
                'cmd': f'task:{name}', 'success': outcome == 'done',
                'elapsed': round(_t.monotonic() - start, 2), 'msg': outcome})

    def use_budget(self, total_s: float = 900.0, *, reserve_s: float = 45.0):
        """Ration the run by the clock. OPT-IN: nothing is rationed until this is called.

        The clock starts on a SUCCESSFUL `arm()`, not on script start -- time
        spent waiting for a tether to come off is not run time. `reserve_s` is
        the surface-and-disarm allowance and is never offered to a task.
        Returns the `RunBudget` so a mission can also call `plan()` on it.

            mongla.use_budget(900, reserve_s=60)
            v = mongla.worth_attempting('torpedo', points=300, worst_case_s=120,
                                        fallback_s=25, fallback_points=100)
            if v.mode == 'full': ...
            elif v.mode == 'fallback': ...          # e.g. blind fire
        """
        from mongla_planner.run_budget import RunBudget
        self._budget = RunBudget(float(total_s), reserve_s=float(reserve_s))
        self.log.info(f'[BUDG ] run budget ON: {total_s:.0f} s total, '
                      f'{reserve_s:.0f} s reserve, clock starts on arm')
        return self._budget

    def note(self, name: str, msg: str, *, success: bool = True) -> None:
        """Put a mission-level fact on the scoreboard that no verb reports.

        e.g. `note('navigation', 'unconfirmed: blind transit', success=False)` --
        a run that proceeds on an attempt must not read as a confirmed pass.
        """
        (self.log.info if success else self.log.warning)(f'[NOTE ] {name}: {msg}')
        self._scoreboard.append({'cmd': f'note:{name}', 'success': bool(success),
                                 'elapsed': 0.0, 'msg': str(msg)})

    def worth_attempting(self, name: str, *, points: int, worst_case_s: float,
                         fallback_s: float = 0.0, fallback_points: int = 0):
        """`Verdict(attempt, mode='full'|'fallback'|'skip', reason, remaining_s)`.

        With no `use_budget()` the answer is always attempt/full -- the opt-out
        IS not calling `use_budget`. The verdict goes on the scoreboard, so a
        skipped task explains itself after the run.
        """
        from mongla_planner.run_budget import Task, Verdict
        budget = getattr(self, '_budget', None)
        if budget is None:
            v = Verdict(True, 'full', 'no run budget configured', float('inf'))
        else:
            v = budget.verdict(Task(str(name), int(points), float(worst_case_s),
                                    fallback_s=float(fallback_s),
                                    fallback_points=int(fallback_points)))
            (self.log.info if v.attempt else self.log.warning)(
                f'[BUDG ] {name}: {v.mode} -- {v.reason}')
        self._scoreboard.append({'cmd': f'budget:{name}', 'success': bool(v.attempt),
                                 'elapsed': 0.0, 'msg': f'{v.mode}: {v.reason}'})
        return v

    def step(self, name: str, *, points: int, worst_case_s: float,
             run, fallback=None, fallback_s: float = 0.0,
             fallback_points: int = 0):
        """One declared piece of a run. Pairs with `run_plan()`.

        `run` and `fallback` are CALLABLES taking `(mongla)` -- never verb
        names, the same rule `resilience.py` holds: a mechanism that accepts a
        verb name can be pointed at `disarm` by a config edit.
        """
        return {'name': str(name), 'points': int(points),
                'worst_case_s': float(worst_case_s), 'run': run,
                'fallback': fallback, 'fallback_s': float(fallback_s),
                'fallback_points': int(fallback_points)}

    def run_plan(self, steps, *, order: str = 'as_written'):
        """Execute a declared plan, re-deciding from the LIVE clock each time.

        This is the whole point: a mission stops being a hand-ordered script
        that hopes it fits, and becomes a declaration of what each task is worth
        and what it costs. Between steps the clock has moved and the world has
        changed, so the verdict is asked again -- never precomputed once.

            mongla.use_budget(900, reserve_s=60)
            mongla.run_plan([
                mongla.step('gate',   points=100, worst_case_s=120,
                            run=gate, fallback=blind_transit,
                            fallback_s=25, fallback_points=40),
                mongla.step('bins',   points=200, worst_case_s=180, run=bins),
                mongla.step('flares', points=50,  worst_case_s=90, run=flares),
            ])

        What it handles so the author does not:
          * the budget verdict per step (full / fallback / skip),
          * a deadline per step, so an overrun is abandoned rather than eating
            the run -- `TaskAbandoned` is caught here and the plan continues,
          * every outcome on the scorecard.

        `order='by_value'` sorts by points per second first. It ignores where
        the props are -- `by_points_per_second` says so itself -- so it is for
        deciding what to DROP, not the order to swim. Default is as written.

        Returns the list of `(name, outcome)` actually executed.
        """
        from mongla_planner.run_budget import Task, by_points_per_second

        steps = list(steps)
        if order == 'by_value':
            ranked = by_points_per_second([
                Task(s['name'], s['points'], s['worst_case_s'],
                     fallback_s=s['fallback_s'],
                     fallback_points=s['fallback_points']) for s in steps])
            by_name = {s['name']: s for s in steps}
            steps = [by_name[t.name] for t in ranked]
            self.log.info('[PLAN ] order by value: '
                          + ' > '.join(s['name'] for s in steps))

        results = []
        for s in steps:
            name = s['name']

            verdict = self.worth_attempting(
                name, points=s['points'], worst_case_s=s['worst_case_s'],
                fallback_s=s['fallback_s'], fallback_points=s['fallback_points'])

            if not verdict.attempt:
                results.append((name, 'skipped'))
                continue

            use_fallback = verdict.mode == 'fallback'
            body = s['fallback'] if use_fallback else s['run']
            if body is None:
                self.note(name, 'no fallback authored for this step',
                          success=False)
                results.append((name, 'skipped'))
                continue

            deadline = s['fallback_s'] if use_fallback else s['worst_case_s']
            try:
                with self.task(name, deadline_s=deadline):
                    body(self)
                results.append((name, verdict.mode))
            except TaskAbandoned:
                # A task that cannot be finished must not cost the run. The
                # deadline already cancelled the goal in flight; keep swimming.
                results.append((name, 'abandoned'))
            except (MoveFailed, MoveRejected) as exc:
                # Issue #16: One failed verb used to end the run. Only
                # TaskAbandoned was caught, so a stall or one busy reply inside
                # a step left run_plan, the runner disarmed, and the
                # higher-value steps after it never ran -- while the vision
                # verbs shrugged off the very same fault. Same policy now: a
                # step's failure costs that step.
                #
                # Unless the vehicle cannot take a stop. Whether the failure was
                # one task's or the vehicle's is MEASURED, not read from the
                # error text: brake, and if the stop itself is refused or times
                # out, the link or the vehicle is gone -- re-raise and let the
                # runner's shutdown take over. Ctrl-C is not
                # caught here.
                try:
                    self.stop()
                except (MoveFailed, MoveRejected) as stop_exc:
                    self.log.error(f'[PLAN ] {name}: {exc} -- and the vehicle '
                                   f'refused a stop ({stop_exc}): ending the run')
                    raise exc from stop_exc
                self.log.warning(f'[PLAN ] {name} FAILED ({exc}); braked, '
                                 f'moving on to the next step')
                results.append((name, f'failed: {exc}'))

        done = [n for n, o in results if o in ('full', 'fallback')]
        self.log.info(f'[PLAN ] {len(done)}/{len(steps)} steps carried out: '
                      + ', '.join(f'{n}={o}' for n, o in results))
        return results
