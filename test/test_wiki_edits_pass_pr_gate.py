"""Wiki edits must face the same PR checks as code changes.

The old gollum handler committed passing documentation edits directly to main.
That route bypassed pull_request integrity checks and would be blocked by the
branch protection required by #88. The wiki is still the edit surface; the
resulting source change must be proposed for review, not pushed into main.
"""
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
WIKI = ROOT / ".github/workflows/wiki.yml"


def wiki_gate_errors(source: str) -> list[str]:
    try:
        doc = yaml.load(source, Loader=yaml.BaseLoader)
    except yaml.YAMLError as exc:
        return [f"invalid YAML: {exc}"]
    if not isinstance(doc, dict):
        return ["not a YAML workflow"]
    errors: list[str] = []
    if "gollum" not in doc.get("on", {}):
        errors.append("gollum event missing")
    jobs = doc.get("jobs", {})
    pull = jobs.get("pull", {})
    publish = jobs.get("publish", {})
    steps = pull.get("steps", []) if isinstance(pull, dict) else []
    edits = [s for s in steps if isinstance(s, dict)
             and "Propose wiki edit" in s.get("name", "")]
    if len(edits) != 1:
        return errors + ["exactly one wiki proposal step required"]
    script = edits[0].get("run", "")
    if "gh pr create --base main --head" not in script:
        errors.append("wiki changes never open a pull request")
    if "HEAD:refs/heads/" not in script:
        errors.append("wiki changes must be sent to a review branch")
    if "HEAD:main" in source or "HEAD:refs/heads/main" in source:
        errors.append("a gollum handler can still push directly to main")
    if not ("GUARDS_OK" in script and "exit 1" in script):
        errors.append("local wiki guards may be ignored")
    cond = str(publish.get("if", ""))
    if "github.event_name == 'push'" not in cond:
        errors.append("wiki publish must follow approved main push")
    if "github.event_name == 'workflow_dispatch'" not in cond:
        errors.append("manual wiki publishing must remain available")
    if "github.event_name != 'gollum'" in cond or "committed ==" in cond:
        errors.append("unreviewed wiki edit may publish before merge")
    return errors


def test_wiki_workflow_requires_a_pr_for_every_source_change():
    assert WIKI.is_file()
    assert not wiki_gate_errors(WIKI.read_text()), "\n".join(
        wiki_gate_errors(WIKI.read_text())
    )


def test_injection_direct_main_push_is_rejected():
    original = WIKI.read_text()
    bad = original.replace('HEAD:refs/heads/', 'HEAD:main', 1)
    assert bad != original
    assert any("directly to main" in e for e in wiki_gate_errors(bad))


def test_injection_silent_wiki_publish_is_rejected():
    original = WIKI.read_text()
    bad = original.replace(
        "github.event_name == 'push' || github.event_name == 'workflow_dispatch'",
        "github.event_name != 'gollum'", 1
    )
    assert bad != original
    assert any("publish" in e for e in wiki_gate_errors(bad))


def test_injection_missing_pull_request_is_rejected():
    original = WIKI.read_text()
    bad = original.replace("gh pr create --base main --head",
                           "echo PR creation disabled --base main --head", 1)
    assert bad != original
    assert any("pull request" in e for e in wiki_gate_errors(bad))
