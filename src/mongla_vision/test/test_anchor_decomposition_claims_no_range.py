"""width / scale was published as a range and plotted as "range m" (audit
2026-11-24). It is not one; the range is PnP's."""
import math
from types import SimpleNamespace

from mongla_vision import lock_publish


def test_the_decomposition_publishes_no_range():
    sent = []
    me = SimpleNamespace(_decomp_pub=SimpleNamespace(publish=sent.append),
                         _cam='downward', _target_w_m=0.6)
    pose = SimpleNamespace(ok=True, theta=3.0, scale=1.5)
    cls = [v for v in vars(lock_publish).values()
           if isinstance(v, type) and hasattr(v, '_publish_decomposition')][0]
    cls._publish_decomposition(me, pose, None)
    assert sent and sent[0].vector.y == 1.5 and math.isnan(sent[0].vector.z)
