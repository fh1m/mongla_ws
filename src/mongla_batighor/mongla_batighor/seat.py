"""Who is the pilot: Batighor's token, as rules with no ROS in them.

Decided with the owner (bead mongla_ws-102.25.3):

  * One pilot; everyone else watches. `disarm`, `stop`, `surface` and aborting
    a mission are open to every operator, pilot or not.
  * A pilot whose browser goes quiet for `LOST_S` loses the token, and any verb
    they sent by hand that is still running is stopped. A mission is not.
  * To take over, an operator asks; the pilot gives it, or says nothing for
    `ANSWER_S` and it passes anyway.
  * Arming needs the pilot to have ticked both safety confirmations.

`helm.py` is the node around this; it owns the clock and the vehicle.
"""
from __future__ import annotations

from dataclasses import dataclass, field

LOST_S = 3.0              # the console beats every second; three missed is gone
ANSWER_S = 10.0           # how long a pilot has to answer a request
ANYONE = frozenset({'disarm', 'stop', 'surface'})
ARM_CONFIRMS = frozenset({'propellers_clear', 'kill_switch_manned'})


@dataclass
class Verdict:
    ok: bool
    message: str


@dataclass
class Seat:
    pilot: str = ''
    requester: str = ''
    request_deadline: float = 0.0
    last_beat: float = 0.0
    events: list = field(default_factory=list)   # what changed, for the log

    def _note(self, what: str) -> None:
        self.events.append(what)

    def heartbeat(self, operator: str, now: float) -> Verdict:
        if operator and operator == self.pilot:
            self.last_beat = now
            return Verdict(True, 'pilot')
        return Verdict(True, 'watching')

    def request(self, operator: str, now: float) -> Verdict:
        if not operator:
            return Verdict(False, 'type your name first')
        if not self.pilot:
            self._seat(operator, now, 'took the free helm')
            return Verdict(True, 'you are the pilot')
        if operator == self.pilot:
            return Verdict(True, 'you are already the pilot')
        if self.requester and self.requester != operator:
            return Verdict(False, f'{self.requester} asked first')
        self.requester = operator
        self.request_deadline = now + ANSWER_S
        self._note(f'{operator} asked {self.pilot} for the helm')
        return Verdict(True, f'asked {self.pilot}; it passes to you in '
                             f'{ANSWER_S:.0f} s unless they answer')

    def give(self, operator: str, now: float) -> Verdict:
        if operator != self.pilot:
            return Verdict(False, 'only the pilot can give the helm')
        if not self.requester:
            return Verdict(False, 'nobody asked for it')
        self._seat(self.requester, now, f'given by {operator}')
        return Verdict(True, f'{self.pilot} is the pilot')

    def release(self, operator: str, now: float) -> Verdict:
        if operator != self.pilot:
            return Verdict(False, 'you are not the pilot')
        if self.requester:
            self._seat(self.requester, now, f'released by {operator} to the asker')
        else:
            self._note(f'{operator} left the helm')
            self.pilot = ''
        return Verdict(True, 'released')

    def tick(self, now: float) -> bool:
        """Advance the clock. True when the pilot was just lost to silence."""
        if self.pilot and now - self.last_beat > LOST_S:
            gone = self.pilot
            self.pilot = ''
            self._note(f'{gone} went silent for {LOST_S:.0f} s and lost the helm')
            if self.requester:
                self._seat(self.requester, now, 'the pilot was lost')
            return True
        if self.requester and now >= self.request_deadline:
            self._seat(self.requester, now, f'{self.pilot} did not answer')
        return False

    def may_send(self, operator: str, cmd: str, confirms=()) -> Verdict:
        if cmd in ANYONE:
            return Verdict(True, 'open to every operator')
        if not operator or operator != self.pilot:
            who = self.pilot or 'nobody'
            return Verdict(False, f'only the pilot sends {cmd} (pilot: {who}); '
                                  f'disarm, stop and surface are open to all')
        if cmd == 'arm':
            missing = sorted(ARM_CONFIRMS - set(confirms))
            if missing:
                return Verdict(False, 'arm needs both confirmations: '
                                      + ', '.join(missing))
        return Verdict(True, 'pilot')

    def _seat(self, operator: str, now: float, why: str) -> None:
        self.pilot = operator
        self.requester = ''
        self.request_deadline = 0.0
        self.last_beat = now
        self._note(f'{operator} is the pilot ({why})')
