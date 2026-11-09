#!/usr/bin/env python3
"""The fast integrity gate: the one list of guards, run here and in CI.

    python3 tools/run_repo_integrity.py

CI runs exactly this line (`tools/ci_contract.py` refuses any other), so the
list below is the only place a guard is named. Moving or deleting a guard
file fails the gate rather than shrinking it.

Passing pytest is not enough here. A guard that skips itself, or whose file
collects nothing, would pass while checking nothing, so the gate fails on any
skip and on any guard file with no passing test. Full ROS build and test are
a separate job (#65).
"""
from __future__ import annotations

import os
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

GUARDS = (
    'test/test_repository_integrity_workflow.py',
    'test/test_provenance_check.py',
    'test/test_wiki_edits_pass_pr_gate.py',
    'test/test_claims_say_where_they_came_from.py',
    'test/test_no_capability_is_unreachable_anywhere.py',
    'test/test_the_retired_stack_stays_retired.py',
    'test/test_package_xml_declares_imports.py',
    'test/test_tools_point_at_files_that_exist.py',
    'src/mongla_control/test/test_doc_drift.py',
    'src/mongla_vision/test/test_no_capability_is_built_and_unreachable.py',
    'src/mongla_vision/test/test_no_function_is_built_and_uncalled.py',
    'src/mongla_manager/test/test_docs_contract.py',
    'src/mongla_manager/test/test_ledger_claims.py',
)


class _Outcomes:
    """Counts passes per guard file and records every skip."""

    def __init__(self):
        self.passed = Counter()
        self.skipped = []

    def pytest_runtest_logreport(self, report):
        path = report.nodeid.split('::', 1)[0]
        if report.when == 'call' and report.passed:
            self.passed[path] += 1
        if report.skipped:
            self.skipped.append(report.nodeid)


def main(guards=GUARDS, root=ROOT) -> int:
    missing = [g for g in guards if not (root / g).is_file()]
    if missing:
        for g in missing:
            print(f'FAIL guard file missing: {g}')
        return 1
    os.chdir(root)
    # Neither installed plugins nor inherited options (a `-k` in
    # PYTEST_ADDOPTS deselects tests silently) may change what runs.
    os.environ['PYTEST_DISABLE_PLUGIN_AUTOLOAD'] = '1'
    os.environ.pop('PYTEST_ADDOPTS', None)
    import pytest
    seen = _Outcomes()
    rc = int(pytest.main(['-q', '-p', 'no:cacheprovider', *guards], plugins=[seen]))
    problems = [f'FAIL skipped: {n}' for n in seen.skipped]
    problems += [f'FAIL no passing test in {g}' for g in guards if not seen.passed[g]]
    for p in problems:
        print(p)
    if rc == 0 and problems:
        rc = 1
    print('ok' if rc == 0 else f'FAIL (exit {rc})')
    return rc


if __name__ == '__main__':
    sys.exit(main())
