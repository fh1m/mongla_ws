"""Every launch's detector `conf` default is the node's own, `detector_params.CONF`.

Three launches defaulted to 0.35 and two places to 0.45, so the same detector
acted at a different bar depending on how it was started (second-pass audit
F3). The launch files keep a literal because the reference generator, the deck
file and `launch_defaults` read them statically; this compares that literal
with the constant, which the node and the composed process import.
"""
import importlib.util
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src' / 'mongla_vision'))
from mongla_vision.detector_params import CONF  # noqa: E402

_spec = importlib.util.spec_from_file_location('launch_ast', ROOT / 'tools' / 'launch_ast.py')
la = sys.modules.setdefault('launch_ast', importlib.util.module_from_spec(_spec))
_spec.loader.exec_module(la)


def test_every_conf_argument_defaults_to_the_node_bar():
    seen, wrong = [], []
    for lf in sorted((ROOT / 'src' / 'mongla_vision' / 'launch').glob('*.launch.py')):
        for a in la.declarations(lf):
            if a.source == lf.name and a.name in ('conf', 'fwd_conf', 'dwn_conf'):
                seen.append(f'{lf.name}:{a.name}')
                if float(a.default) != CONF:
                    wrong.append(f'{lf.name}: {a.name}={a.default}')
    assert len(seen) >= 4, f'parsed too few conf arguments, the parser has gone stale: {seen}'
    assert not wrong, f'detector_params.CONF is {CONF}; these launches say otherwise: {wrong}'
