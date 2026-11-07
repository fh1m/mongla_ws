"""How every bringup check reports: a grade and one aligned line.
"""
from __future__ import annotations


PASS = 'PASS'


WARN = 'WARN'


FAIL = 'FAIL'


def _line(tag: str, label: str, detail: str = '') -> None:
    print(f'  [{tag}] {label:<34} {detail}')
