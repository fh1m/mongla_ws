"""A raise inside a helm request (no `ros2` on PATH, an unwritable log
folder, mongla_vision absent) left the browser with no reply (audit
2026-11-24, batighor_2 #3). Every request is answered."""
from types import SimpleNamespace

from mongla_batighor.helm import HelmNode


def test_a_request_that_raises_is_answered_not_dropped():
    logged = []

    def boom(_req, _res):
        raise FileNotFoundError('ros2')
    me = SimpleNamespace(_handle=boom, _log=lambda **kw: logged.append(kw))
    req = SimpleNamespace(who=' ana ', op='mission', cmd='')
    res = SimpleNamespace(ok=True, message='')
    out = HelmNode._on_request(me, req, res)
    assert out is res and res.ok is False and 'ros2' in res.message
    assert logged and logged[0]['ok'] is False
