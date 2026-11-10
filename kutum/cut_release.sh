#!/usr/bin/env bash
# Cut a GitHub release for a vehicle deploy whose own gate passed (#96).
#
#   kutum/cut_release.sh GATE_EXIT DIRTY RELEASE_JSON
#
# Run on the dev box, from the repository, by kutum/deploy.sh after the
# vehicle's gate. A release names a configuration someone could fly again, so
# it is cut only when all of these hold:
#
#   - the vehicle's own gate (bringup_check) exited 0;
#   - the tree had no uncommitted changes (DIRTY is empty);
#   - HEAD is on origin/main, so the tag points at reviewed code.
#
# The asset is that release's release.json: the commit, the firmware floor,
# model, calibration and hull hashes, apt, pip and HailoRT versions, and the
# gate verdict. Every refusal prints its reason and exits 0: the vehicle is
# provisioned either way, and a missing release must never fail a deploy.
set -euo pipefail

gate="${1:?gate exit code}" dirty="${2-}" manifest="${3:?path to release.json}"
say() { printf '  %s\n' "$*"; }

if [[ "$gate" != 0 ]]; then
  say "no release: the vehicle gate refused (exit $gate)"; exit 0
fi
if [[ -n "$dirty" ]]; then
  say "no release: the tree had uncommitted changes"; exit 0
fi
if [[ ! -s "$manifest" ]]; then
  say "no release: no release.json came back from the vehicle"; exit 0
fi
full="$(git rev-parse HEAD)"
if ! git fetch -q origin main 2>/dev/null; then
  say "no release: origin is unreachable, so main cannot be checked"; exit 0
fi
if ! git merge-base --is-ancestor "$full" origin/main; then
  say "no release: ${full:0:8} is not on main"; exit 0
fi
if ! command -v gh > /dev/null; then
  say "no release: gh is not installed"; exit 0
fi

tag="vehicle-$(date -u +%Y%m%d)-${full:0:8}"
notes="$(python3 - "$manifest" <<'EOF'
import json, sys
d = json.load(open(sys.argv[1]))
print(f"Built, active and gate-passed on {d.get('host', 'Kutum')} at "
      f"{d.get('vehicle_gate_utc', '?')}: {d.get('commit', '?')}.\n")
print(f"- board firmware behaviour revision required: "
      f"{d.get('board_firmware_rev_required')}")
print(f"- models: {', '.join(sorted(d.get('models') or {})) or 'none in the tree'}")
print(f"- calibration: {', '.join(sorted(d.get('calibration') or {})) or 'none'}")
print(f"- ROS {d.get('ros_distro')}, {d.get('hailortcli') or 'no HailoRT'}")
print('\nrelease.json is the whole bill of materials, with every hash. A gate '
      'pass is a bench verdict, not a water test (docs/evidence/lineage.md).')
EOF
)"
if gh release create "$tag" --target "$full" --title "$tag" --notes "$notes" \
     "$manifest#release.json"; then
  say "released $tag"
else
  say "WARNING: could not cut $tag (it may already exist); the deploy stands"
fi
