#!/usr/bin/env bash
# Copy what lives only on Kutum to the dev box, so a dead SD card costs a
# morning instead of a model.
#
#   kutum/backup.sh fh1m@mongla.local              # the irreplaceable set
#   kutum/backup.sh fh1m@mongla.local --runs       # ... and every run and capture
#   KUTUM_BACKUP=/elsewhere kutum/backup.sh ...    # default ~/kutum-backup/<host>
#
# The repo rebuilds everything else: kutum/setup.sh the machine, deploy.sh the
# workspace. What it cannot rebuild is what was made ON the vehicle:
#
#   ~/hailo_models   compiled XFeat artifacts. On 2026-11-07 these existed
#                    nowhere but the vehicle.
#   ~/.mongla        the deck's own config: bringup.yaml, courses, geometry
#   ~/calib_*        the calibration captures the guide resumes from
#
# --runs adds ~/mongla_runs and ~/data (bags, logs, clips; gigabytes).
#
# Pull-only and additive: it never deletes on either side, so a backup taken
# from a half-wiped machine cannot erase the good copy.
set -euo pipefail

TARGET="${1:?usage: $0 user@host [--runs]}"
host="${TARGET#*@}"; host="${host%%.*}"
DEST="${KUTUM_BACKUP:-$HOME/kutum-backup}/$host"
mkdir -p "$DEST"

what=(hailo_models .mongla 'calib_*')
[ "${2:-}" = "--runs" ] && what+=(mongla_runs data)

for w in "${what[@]}"; do
  # A glob is expanded on the vehicle; a name that is absent there is skipped.
  for src in $(ssh -o BatchMode=yes "$TARGET" "cd ~ && ls -d $w 2>/dev/null"); do
    rsync -a --info=stats0 "$TARGET:~/$src" "$DEST/"
    echo "  ok   ~/$src -> $DEST/$src ($(du -sh "$DEST/$src" | cut -f1))"
  done
done
date -u +%FT%TZ > "$DEST/.backed-up"
echo "Backed up to $DEST"
