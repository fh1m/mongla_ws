# kutum/

Kutum is the vehicle's compute module: a Raspberry Pi 5 with a Hailo-8 AI HAT+, Ubuntu
24.04 and ROS 2 Jazzy. This folder is that machine, as files. Why it exists, what is on it
and how it is laid out:
[the Kutum chapter](../.claude/context/kutum/README.md).

![Kutum, layer by layer](../docs/imgs/readme/kutum-stack.webp)

## The files

```
kutum/
  setup.sh            make a Pi into Kutum            (--system: the machine; plain: your account)
  deploy.sh           put this tree on it, build, gate   (run from the dev box)
  release.sh          releases on the vehicle: build, switch, roll back (deploy.sh drives it)
  release_manifest.py what a release is: commit, models, calibration, packages
  armed_probe.py      is the board armed? read-only; deploy refuses if it cannot tell
  doctor.sh           is it still Kutum?                 (read-only, exits 1 on FAIL)
  backup.sh           pull what only the vehicle has     (run from the dev box)
  env.sh              what every terminal on the vehicle sources
  requirements.txt    the pip packages, pinned to what runs on the vehicle
  constraints.txt     numpy < 2, applied to every pip install
  system/
    udev/99-mongla-cameras.rules   stable camera names by USB port
    udev/51-hailo-udev.rules       /dev/hailo0 readable without root
    udev/make_srot_rule.sh         pin the SROT board to /dev/srot (once per board)
    systemd/cpu-performance.service + set-cpu-performance   CPU governor at boot
    netplan/60-kutum-eth.yaml      Ethernet: DHCP plus a link-local fallback
```

## The network

Kutum answers as **`mongla.local`** on every interface (avahi), so the address
never matters:

| Link | How it gets an address |
|---|---|
| Ethernet to a router | DHCP |
| Ethernet straight to a laptop sharing its internet | DHCP from the laptop. On a NetworkManager laptop: `nmcli connection modify "<wired connection>" ipv4.method shared`, then reconnect |
| Ethernet straight to a laptop, no sharing | a link-local address (169.254.x.x); `mongla.local` still resolves |
| Wi-Fi | joins the network `setup.sh --system` asked for, on every boot when it is in range |

The Wi-Fi password is typed into `setup.sh --system` by whoever runs it and
lives only in `/etc/netplan/61-kutum-wifi.yaml` (root, mode 0600). To change
networks: `sudo rm /etc/netplan/61-kutum-wifi.yaml`, then rerun
`sudo ~/mongla_ws/kutum/setup.sh --system`.

## The clock

The Pi 5 has a real-time clock; the official RTC battery keeps it running
through a power-off, so a vehicle booted at a pool with no network still knows
the date. `setup.sh --system` turns on the battery's trickle charge
(`dtparam=rtc_bbat_vchg=3000000` in `/boot/firmware/config.txt`; rechargeable
ML-2020 cells only). The doctor reads the charge setting, the cell's own
voltage (`vcgencmd pmic_read_adc BATT_V`; below 2.5 V is a WARNING), whether the
RTC agrees with the system clock, and whether network time has synchronised
since boot. Timing measured on Kutum, and why it stays on `systemd-timesyncd`:
measured-bars §68.

## Every day

```bash
kutum/deploy.sh fh1m@mongla.local               # after any change: sync, doctor, build, gate
kutum/deploy.sh fh1m@mongla.local --check-only  # the doctor, nothing changed
kutum/deploy.sh fh1m@mongla.local --list        # the releases on the vehicle
kutum/deploy.sh fh1m@mongla.local --rollback    # back to the release before this one
kutum/backup.sh fh1m@mongla.local               # after anything is made on the vehicle
kutum/backup.sh fh1m@mongla.local --runs        # ... and the runs and footage (gigabytes)
ssh fh1m@mongla.local bash -s < kutum/doctor.sh # the doctor, from here
```

On the vehicle, every shell already has ROS and the workspace (`env.sh`), so:

```bash
~/mongla_ws/kutum/doctor.sh
ros2 run mongla_manager bringup_check
```

### Releases

Every deploy is a new release, `~/mongla/releases/<commit>-<time>/`, and `~/mongla_ws` is
a symlink to the active one, so every path and habit keeps working. In order, a deploy:

1. **refuses before it writes anything** if a manager, a launch, a mission or the planner
   is running, or if the board says ARMED, or if a board is attached and cannot be read.
   Stop the stack with Ctrl-C, so it disarms, and rerun;
2. stages the new tree beside the live one (unchanged files are hard links, so a release
   costs what changed), runs the doctor from it, and builds it from clean in place;
3. switches only if the build succeeded, with one rename. A failed build, a cut tether or
   a doctor FAIL leaves the active release exactly as it was;
4. writes `release.json` (the commit, the firmware floor the code demands, every model
   and calibration by hash, the hull geometry, the apt and pip versions under it), keeps
   the five newest releases, and records the gate's verdict in the manifest;
5. cuts a GitHub release, `vehicle-<date>-<commit>`, with that `release.json` attached
   (`cut_release.sh`), but only when the gate passed, the tree was clean and the commit
   is on `main`. Otherwise it prints why and skips it; a missing release never fails a
   deploy, and a gate pass is a bench verdict, not a water test.

`--rollback` goes back to the previous release after the same safety checks, and again
goes forward. The first deploy keeps the old plain workspace as `legacy-<time>`; it is
never pruned and never a rollback target, because its `install/` was built at the old path.
On the vehicle, `bash ~/mongla_ws/kutum/release.sh list` shows them all.

A release is never edited in place. A model or a calibration copied into one vanishes
with it: put it in the dev box's tree and deploy again.

The doctor's three words: `ok` as declared, `WARNING` runs but worse than declared,
`FAIL` the stack cannot run here. Deploy stops on a FAIL and builds through a WARNING.

## When Kutum burns

The repo rebuilds everything except what was made on the vehicle, and `backup.sh` keeps
that. From a blank SD card to a gated vehicle:

1. **Flash** Ubuntu Server 24.04 LTS (64-bit) for the Raspberry Pi with Raspberry Pi
   Imager. In its settings: hostname `mongla`, user `fh1m`, your SSH public key.
2. **Seat the AI HAT+** and its PCIe ribbon. Boot on Ethernet, then
   `ssh fh1m@mongla.local`.
3. **Install ROS 2 Jazzy** `ros-base`, following the official
   [Ubuntu (deb packages) instructions](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
4. From the dev box: `kutum/deploy.sh fh1m@mongla.local`. It stages the tree, then the
   doctor stops it, because there is no Hailo stack yet. That stop is expected, and it
   prints the staged path.
5. On the vehicle: `sudo ~/mongla/releases/<staged>/kutum/setup.sh --system`, with the
   path the doctor printed (nothing is active yet, so `~/mongla_ws` does not exist). This builds the Hailo
   driver, firmware and runtime from source (the runtime build is the long part), installs
   `system/`, gives `fh1m` passwordless sudo, runs the user half, and ends with the doctor.
   Its exit code is the doctor's verdict: `0` ready; `1` a FAIL, fix it and rerun; `3` a
   FAIL while a change waits for a reboot, so reboot and run `kutum/doctor.sh`, and a FAIL
   then is real. Log out and back in, so the `dialout` group applies.
6. From the dev box, put back what only the old vehicle had:
   `rsync -a --exclude .backed-up ~/kutum-backup/mongla/ fh1m@mongla.local:~/`
7. From the dev box: `kutum/deploy.sh fh1m@mongla.local`. The detector models travel with
   the tree (they are untracked files in `src/mongla_vision/models/`, so they come from
   the dev box's copy). It builds and runs `bringup_check`.
8. **Plug in the board** and pin it once:
   `~/mongla_ws/kutum/system/udev/make_srot_rule.sh`.

`setup.sh --system` also initialises rosdep, and the doctor runs `rosdep check`
against the package.xml files, so a dependency the code imports and nothing
declares shows up before it is an ImportError.

**State:** steps 4, 6 and 7 RUN TODAY. Step 5 is the hand-built procedure of 2026-08-28
written as one script, and has not yet run on a blank card. When it does, note what broke
here.

## Security, on purpose

`setup.sh --system` writes `/etc/sudoers.d/90-kutum-fh1m` with `NOPASSWD: ALL`. Kutum is
a single-purpose machine on the vehicle's own network, and a password prompt in the middle
of a pool session costs more than it protects (the owner's call, 2026-11-07). Delete that
file to undo it. `deploy.sh` and `backup.sh` run ssh in `BatchMode`: they need a key
(`ssh-copy-id fh1m@mongla.local`, once) and never prompt for a password.

What each job actually needs (GitHub #67). Only provisioning and machine
maintenance use root; nothing on the mission path does.

| job | runs as | needs |
|---|---|---|
| provisioning (`setup.sh --system`) | root, through sudo | everything it installs |
| OS update, shutdown, a system file | root, through sudo | the operator's `NOPASSWD` |
| deploy, build (`deploy.sh`) | `fh1m`, over a key | the workspace; no sudo |
| ROS launch, mission, recording | `fh1m` | `dialout` for the board (setup adds it), the camera nodes, the Hailo node (udev makes it 0666) |
| model switching | `fh1m` | `~/models`; no sudo |
| doctor | `fh1m` | read-only; `sudo -n` only to compare the root-only netplan file |
| emergency stop | nobody | the board's kill switch, and Ctrl-C on the manager; no privilege on the path |

The one root unit Kutum installs, `cpu-performance.service`, is a oneshot that
sets the CPU governor and exits; it listens on nothing. A unit that listens on
the network (a future deck service) runs as its own user, never root, with
`NoNewPrivileges=`, `ProtectSystem=strict` and `PrivateTmp=`, so a compromised
service cannot reach the operator's sudo. `test_kutum.py` refuses a unit that
does not. The operator account keeps `NOPASSWD` (the owner's call, 2026-11-09).

The mission web console (`ros2 launch mongla_vision mission_web.launch.py`)
listens on every interface with no login, and its POST routes change detector
parameters and the active camera. That is deliberate (the owner's call,
2026-11-09, GitHub #66): at the pool everyone on the vehicle's network is the
team, and any of them may need to change something from any device. It is off
unless launched, and it never touches the board, which only `auv_manager_node`
owns. Revisit this if the console is ever served beyond the team's network.
