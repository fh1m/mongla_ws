"""The mandatory fast CI gate must actually run the existing anti-drift guards.

The project has extensive orphan/documentation checks but until this workflow
they were not required to execute on pull requests. A guard without a real
GitHub Actions caller is itself an unintegrated capability. These tests examine
the parsed workflow (not a comment or README promise), then inject regressions.

Full ROS Jazzy/Humble build/integration remains a separate requirement, #65.
"""
from __future__ import annotations

import shlex
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
WORKFLOW = ROOT / ".github/workflows/repository-integrity.yml"

# Real source/documentary entrypoint tests. If any test file moves, update this
# contract and CI in the SAME change; a missing test cannot silently disappear.
REQUIRED_TESTS = frozenset({
    "test/test_repository_integrity_workflow.py",
    "test/test_claims_say_where_they_came_from.py",
    "test/test_no_capability_is_unreachable_anywhere.py",
    "test/test_the_retired_stack_stays_retired.py",
    "test/test_package_xml_declares_imports.py",
    "test/test_tools_point_at_files_that_exist.py",
    "src/mongla_control/test/test_doc_drift.py",
    "src/mongla_vision/test/test_no_capability_is_built_and_unreachable.py",
    "src/mongla_vision/test/test_no_function_is_built_and_uncalled.py",
    "src/mongla_manager/test/test_docs_contract.py",
    "src/mongla_manager/test/test_ledger_claims.py",
})


def contract_errors(workflow_text: str) -> list[str]:
    """Validate the actual YAML job and command tokens, never comments."""
    try:
        doc = yaml.load(workflow_text, Loader=yaml.BaseLoader)
    except yaml.YAMLError as exc:
        return [f"invalid workflow YAML: {exc}"]
    if not isinstance(doc, dict):
        return ["workflow must be a mapping"]
    errors = []
    triggers = doc.get("on")
    if not isinstance(triggers, dict):
        triggers = {}
    for trigger in ("pull_request", "push"):
        if trigger not in triggers:
            errors.append(f"missing required event {trigger!r}")
    if doc.get("permissions", {}).get("contents") != "read":
        errors.append("workflow requires permissions: contents: read")
    job = doc.get("jobs", {}).get("source-integrity")
    if not isinstance(job, dict):
        return errors + ["missing source-integrity job"]
    if job.get("continue-on-error") == "true":
        errors.append("job failure must not be ignored")
    if job.get("if") in ("false", "${{ false }}"):
        errors.append("job must not be disabled")
    steps = job.get("steps", [])
    if not isinstance(steps, list):
        return errors + ["source-integrity must have steps"]
    tests = set()
    saw_pytest = False
    for step in steps:
        if not isinstance(step, dict):
            continue
        if step.get("continue-on-error") == "true":
            errors.append("step may not ignore failures")
        shell = str(step.get("run", ""))
        if not shell:
            continue
        if "|| true" in shell or "|| :" in shell:
            errors.append("shell command may not convert test failure to success")
        try:
            tokens = shlex.split(shell, comments=True)
        except ValueError as exc:
            errors.append(f"invalid shell syntax: {exc}")
            continue
        if "-m" in tokens and "pytest" in tokens:
            saw_pytest = True
            tests.update(t for t in tokens if t.endswith(".py"))
            if "--continue-on-collection-errors" in tokens:
                errors.append("collection errors must fail")
    if not saw_pytest:
        errors.append("no executable python -m pytest command")
    for rel in sorted(REQUIRED_TESTS - tests):
        errors.append(f"required guard absent from pytest invocation: {rel}")
    return errors


def test_actual_workflow_really_executes_every_guard():
    assert WORKFLOW.is_file()
    assert not contract_errors(WORKFLOW.read_text()), "\n".join(
        contract_errors(WORKFLOW.read_text())
    )
    missing = [p for p in REQUIRED_TESTS if not (ROOT / p).is_file()]
    assert not missing, f"guard removed from repository: {sorted(missing)}"


def test_injection_a_removed_orphan_guard_makes_ci_contract_fail():
    original = WORKFLOW.read_text()
    bad = original.replace(
        "            test/test_no_capability_is_unreachable_anywhere.py \\\n", "", 1
    )
    assert bad != original
    assert any("test_no_capability_is_unreachable_anywhere.py" in e
               for e in contract_errors(bad))


def test_injection_a_disabled_pull_request_trigger_is_detected():
    original = WORKFLOW.read_text()
    bad = original.replace("  pull_request:\n", "  workflow_dispatch:\n", 1)
    assert bad != original
    assert any("pull_request" in e for e in contract_errors(bad))


def test_injection_a_suppressed_test_failure_is_detected():
    original = WORKFLOW.read_text()
    bad = original.replace(
        "      - name: Run code reachability, docs, provenance and CI-contract guards\n",
        "      - name: Run code reachability, docs, provenance and CI-contract guards\n"
        "        continue-on-error: true\n", 1
    )
    assert bad != original
    assert any("ignore failures" in e for e in contract_errors(bad))


def test_ai_provenance_rechecks_when_pr_body_is_edited():
    """A contributor can repair a missing AI Provenance section without a code push.

    A CI check required by branch protection that only runs on 'opened' and
    'synchronize' stays permanently red after the PR description is fixed.
    """
    policy = ROOT / ".github/workflows/poc-check.yml"
    assert policy.is_file()
    doc = yaml.load(policy.read_text(), Loader=yaml.BaseLoader)
    events = doc["on"]["pull_request"]["types"]
    assert {"opened", "synchronize", "reopened", "edited"} <= set(events)
