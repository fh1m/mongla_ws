"""Flow's scale error is one measurement, stated in two packages.

mongla_vision publishes flow's covariance with `FLOW_SCALE_SIGMA_FRAC`; this
package floors the filter's position sigma with `FLOW_SCALE_SIGMA`. Both are
the worst taped slide, 3.6 % (measured-bars 13), and the vision copy is tied
to the ledger by its own test. Localization does not import mongla_vision
(cv2 comes with it), so this reads the source. Until 2026-11-20 they were
0.036 and 0.035, the second rounded by hand.
"""
import re
from pathlib import Path

from mongla_localization.inekf import FLOW_SCALE_SIGMA

VISION = (Path(__file__).resolve().parents[2] / 'mongla_vision' / 'mongla_vision'
          / 'flow' / 'flow_constants.py')


def test_the_filter_allows_the_scale_error_flow_reports():
    m = re.search(r'^FLOW_SCALE_SIGMA_FRAC = ([0-9.]+)', VISION.read_text(), re.M)
    assert m, f'FLOW_SCALE_SIGMA_FRAC not found in {VISION}'
    assert FLOW_SCALE_SIGMA == float(m.group(1))
