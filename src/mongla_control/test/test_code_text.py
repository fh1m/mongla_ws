"""`tools/code_text.py`: what the source-text guards read (issue #22)."""
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[3] / 'tools'))
from code_text import code_only  # noqa: E402

_SRC = '''
def f(x):
    """Mentions register('allocator') in a docstring."""
    # self._health.register('allocator', g)
    y = g(x = 1)          # aligned and commented
    return y
'''


def test_a_commented_out_call_is_absent():
    assert "register('allocator'" not in code_only(_SRC)


def test_a_docstring_cannot_satisfy_a_guard():
    assert 'Mentions' not in code_only(_SRC)


def test_spacing_cannot_dodge_a_not_in_check():
    assert 'g(x=1)' in code_only(_SRC)


def test_string_literals_that_are_code_survive():
    assert "'near'" in code_only("cfg = {'variant': 'near'}")


def test_an_indented_method_body_parses():
    assert 'return 1' in code_only('    def m(self):\n        return 1\n')
