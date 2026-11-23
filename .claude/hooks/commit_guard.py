#!/usr/bin/env python3
"""PreToolUse hook: stop commits and pushes that break the authorship and hygiene rules.

Deterministic and fast: no model call, only a few local git reads.

Blocks (exit 2) when:
  * the command text, the staged diff, or the commits a push would send carry a
    `Co-authored-by:` trailer (the user is the sole author);
  * a staged Markdown file adds a machine-local home path (/home/<user>/ or
    /Users/<user>/), which must not reach docs or the public site;
  * the command, the staged diff or a pushed commit carries an assistant session
    URL (a claude.ai code-session address). The repositories are public and the owner
    ruled these out of every commit, PR and doc on 2026-11-24.

Limits, on purpose: it sees only what is already staged when the commit runs, so a
`git add ... && git commit` chain is checked on the staged set at that moment, and
it does not scan code for paths. Both are still caught by the integrity tests.
"""
import json
import os
import re
import subprocess
import sys

ROOT = os.environ.get("CLAUDE_PROJECT_DIR", os.getcwd())
COAUTHOR = re.compile(r"co-authored-by:", re.IGNORECASE)
HOME_PATH = re.compile(r"/(?:home|Users)/[A-Za-z0-9._-]+/")
SESSION_LINK = re.compile(r"claude\.ai/code/session", re.IGNORECASE)


def git(*args):
    try:
        out = subprocess.run(["git", *args], cwd=ROOT, capture_output=True,
                             text=True, timeout=5)
        return out.stdout if out.returncode == 0 else ""
    except Exception:
        return ""  # git unavailable -> do not block on a guess


def block(reason):
    sys.stderr.write(f"[Hook] BLOCKED: {reason}\n"
                     "  -> the user is the sole author: remove the trailer, path or link, then retry.\n")
    sys.exit(2)


def main():
    try:
        payload = json.load(sys.stdin)
    except Exception:
        return  # malformed payload -> do not block
    cmd = (payload.get("tool_input") or {}).get("command", "") or ""
    is_commit = re.search(r"\bgit\s+commit\b", cmd)
    is_push = re.search(r"\bgit\s+push\b", cmd)
    if not (is_commit or is_push):
        return

    if COAUTHOR.search(cmd):
        block("the command text carries a Co-authored-by trailer.")
    if SESSION_LINK.search(cmd):
        block("the command text carries an assistant session link.")

    if is_commit:
        staged = git("diff", "--cached", "-U0")
        added = [l[1:] for l in staged.splitlines() if l.startswith("+") and not l.startswith("+++")]
        if any(COAUTHOR.search(l) for l in added):
            block("the staged diff adds a Co-authored-by trailer.")
        if any(SESSION_LINK.search(l) for l in added):
            block("the staged diff adds an assistant session link.")
        md_files = re.findall(r"^\+\+\+ b/(.+\.md)$", staged, re.MULTILINE)
        if md_files:
            md_added = []
            current = None
            for line in staged.splitlines():
                if line.startswith("+++ b/"):
                    current = line[6:]
                elif line.startswith("+") and not line.startswith("+++") and current and current.endswith(".md"):
                    md_added.append(line[1:])
            if any(HOME_PATH.search(l) for l in md_added):
                block("a staged Markdown file adds a machine-local home path.")

    if is_push:
        log = git("log", "@{u}..HEAD", "--format=%B") or git("log", "origin/main..HEAD", "--format=%B")
        if COAUTHOR.search(log):
            block("a commit this push would send carries a Co-authored-by trailer.")
        if SESSION_LINK.search(log):
            block("a commit this push would send carries an assistant session link.")


if __name__ == "__main__":
    main()
    sys.exit(0)
