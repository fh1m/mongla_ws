# Security

## Reporting a vulnerability

Report it privately through GitHub: the **Security** tab of this repository,
**Report a vulnerability**. Only the maintainers see it. Please do not open a
public issue for it.

Useful in a report: what an attacker needs (network access to the vehicle, a
pull request, a crafted file), what they get, and the commit you read.

## What is in scope

- **The CI and repository gates** (`.github/workflows/`, `tools/ci_contract.py`,
  `tools/provenance_check.py`): anything that lets a change merge without the
  required checks running, or runs pull request text as code.
- **The vehicle's computer** (`kutum/`): services, sudo rules, anything that
  reaches the board or the operator's account from the network.
- **The board link** (`src/mongla_control/fc/`, `src/mongla_manager/`): anything
  that arms, moves or keeps the vehicle armed without an operator's verb, or
  defeats a safety latch.

## Deliberately open, not a vulnerability

The mission web console listens on every interface with no login. At the pool
everyone on the vehicle's network is the team, and any device must be able to
change things (owner decision, 2026-11-09; `kutum/README.md`, "Security, on
purpose"). It is off unless launched, and it never talks to the board. If it is
ever reachable beyond the team's network, that is in scope.

## What we turn on

Secret scanning with push protection, Dependabot alerts and security updates,
CodeQL analysis, and actions pinned to commit SHAs. Dependency updates are
proposed weekly and never merge by themselves (`.github/dependabot.yml`).
