"""A paused detector must announce itself.

⛔ WHY. `paused` makes the worker consume each frame and publish nothing. The
camera runs, `image_raw` flows, the `/detections` publisher is registered and
live, and zero messages ever appear -- every liveness check this project owns
passes while the pipeline does nothing.

`vision_pi.launch.py` declares `paused` with default_value='true', so that is
the SHIPPED state until an operator overrides it. On the vehicle (B-XF1) it was
mistaken in turn for a broken model, a wrong confidence, an empty class
allowlist and a dead worker thread -- because the one thing the node never
mentioned was that it had been told not to work.
"""
import ast
from pathlib import Path

NODE = (Path(__file__).resolve().parents[1] / 'mongla_vision'
        / 'detector_node.py')
LAUNCH = (Path(__file__).resolve().parents[1] / 'launch'
          / 'vision_pi.launch.py')


def test_the_paused_branch_warns():
    src = NODE.read_text()
    i = src.index("if self.get_parameter('paused').value:")
    window = src[i:i + 1200]
    assert '.warn(' in window, (
        'the paused branch drops every frame without a word; a silent '
        'detector is indistinguishable from a broken one')
    assert 'PAUSED' in window


def test_the_launch_default_is_still_what_the_warning_claims():
    """If the default ever flips to false, the warning's reasoning is stale and
    should be re-read rather than silently kept."""
    src = LAUNCH.read_text()
    i = src.index("'paused', default_value=")
    assert "default_value='true'" in src[i:i + 60], (
        'paused no longer defaults to true -- revisit the warning text and '
        'B-XF1, which both cite that default')
