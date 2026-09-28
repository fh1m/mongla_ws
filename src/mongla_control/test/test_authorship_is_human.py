"""The contributor list is two people. Keep it that way.

On 2026-09-28 this repository's GitHub contributor list showed four entries: the
two humans who wrote it, plus `claude` and `cursoragent`. Neither agent was ever
an author or committer -- both were pulled in by `Co-Authored-By:` trailers in
commit messages, which GitHub resolves to real accounts. 729 trailers across
1 165 commits, plus 20 commits on two branches whose author field was the agent
itself.

History was rewritten to remove all of it. These tests fail if any of it comes
back, because the next occurrence arrives one commit at a time and nobody
re-reads the contributor sidebar.

The first lock is `includeCoAuthoredBy: false` in `.claude/settings.json`, which
is checked in so it holds on every clone. This file is the second lock: a
setting can be overridden, and a different tool writes different trailers.
"""

import json
import pathlib
import re
import subprocess

import pytest

# Agent identities, not people. Matched against author, committer and trailers.
AGENTS = re.compile(
    r"claude|anthropic|cursor|copilot|codex|devin|chatgpt|openai|gemini|"
    r"windsurf|aider",
    re.IGNORECASE,
)

TRAILER = re.compile(
    r"^[ \t]*co-authored-by:[ \t]*(?P<who>.+)$", re.IGNORECASE | re.MULTILINE
)

# `CLAUDE.md` and `.claude/` are tooling this repo legitimately uses, and a
# subject line may name them. Only trailers and identity fields claim authorship.


def _git(*args: str) -> str:
    return subprocess.run(
        ["git", *args], capture_output=True, text=True, check=True
    ).stdout


def _is_git_repo() -> bool:
    try:
        _git("rev-parse", "--git-dir")
        return True
    except (subprocess.CalledProcessError, FileNotFoundError):
        return False


pytestmark = pytest.mark.skipif(
    not _is_git_repo(), reason="not a git checkout (sdist or vendored copy)"
)


def test_no_agent_appears_as_author_or_committer():
    """Every commit on every branch was authored and committed by a person."""
    lines = _git("log", "--all", "--format=%H%x00%an <%ae>%x00%cn <%ce>").splitlines()
    offenders = [
        (sha, author, committer)
        for sha, author, committer in (ln.split("\0") for ln in lines if ln)
        if AGENTS.search(author) or AGENTS.search(committer)
    ]
    assert not offenders, (
        f"{len(offenders)} commit(s) name an agent as author or committer. "
        f"First: {offenders[0]}. Rewrite with `git-filter-repo --mailmap`, "
        "mapping the agent identity to the person who ran it."
    )


def test_no_agent_is_credited_as_co_author():
    """No commit message carries an agent Co-Authored-By trailer.

    GitHub resolves these to real accounts and lists them as contributors, which
    is exactly how `claude` and `cursoragent` appeared on this repository.
    """
    offenders = [
        who
        for who in TRAILER.findall(_git("log", "--all", "--format=%B"))
        if AGENTS.search(who)
    ]
    assert not offenders, (
        f"{len(offenders)} agent Co-Authored-By trailer(s) are back. "
        f"Distinct: {sorted(set(offenders))}. "
        "Check `includeCoAuthoredBy: false` is still in .claude/settings.json, "
        "then strip them with `git-filter-repo --message-callback`."
    )


def test_attribution_stays_disabled_in_checked_in_settings():
    """The setting that prevents new trailers is checked in, not merely local."""
    root = pathlib.Path(_git("rev-parse", "--show-toplevel").strip())
    settings = json.loads((root / ".claude" / "settings.json").read_text())
    assert settings.get("includeCoAuthoredBy") is False, (
        "`includeCoAuthoredBy: false` is missing from the checked-in "
        ".claude/settings.json. Without it, every clone of this repo starts "
        "writing agent trailers again."
    )
