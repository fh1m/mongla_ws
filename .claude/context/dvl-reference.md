# DVL Reference — Nortek Nucleus 1000

> Implementation files:
> - `src/duburi_sensors/duburi_sensors/sources/nucleus_dvl.py` — connection, auth, integrator
> - `src/duburi_sensors/duburi_sensors/sources/nucleus_parser.py` — binary packet decoder
> - `src/duburi_sensors/duburi_sensors/sources/composite_bno_dvl.py` — BNO085 heading + Nucleus DVL position

---

## 1. Hardware

| Field            | Value                                      |
|------------------|--------------------------------------------|
| Model            | Nortek Nucleus 1000                        |
| Protocol         | Binary over TCP                            |
| IP               | `192.168.2.201` (AUV internal switch)      |
| Port             | `9000`                                     |
| Password         | `nortek`                                   |
| Connect timeout  | 5 s                                        |
| Stale threshold  | 3 s (no packets → `is_healthy()` = False)  |
| Body-frame axes  | x=forward (+), y=right (+), z=up (+)       |
| AHRS convention  | 0–360° NED (0=North, +CW)                  |

---

## 2. Binary Packet Format

All packets share the same header structure (little-endian):

```
Offset  Size   Field
[0]     1      sync byte = 0xa5
[1]     1      sizeHeader (total header length in bytes)
[2]     1      id (packet type, see table below)
[3]     1      family = 0x20 (NUCLEUS)
[4:6]   2      sizeData (uint16)
[6:8]   2      dataCheckSum (uint16)
[8:10]  2      headerCheckSum (uint16)
[sizeHeader : sizeHeader+sizeData]  data payload
```

Checksum algorithm: XOR-sum starting at `0xb58c`, summing pairs of bytes as
uint16 LE. Odd-length data zero-pads the last byte. See `nucleus_parser._checksum()`.

### Packet types

| ID     | Name        | Status in Mongla |
|--------|-------------|-----------------|
| `0xb4` | BottomTrack | **Parsed** — velocity + validity |
| `0xd2` | AHRS        | **Parsed** — heading/roll/pitch  |
| `0xbe` | WaterTrack  | Not parsed — future use (water current velocity) |
| `0xaa` | Altimeter   | Not parsed — future use (distance to seabed) |

---

## 3. BottomTrack Packet (0xb4) — Velocity

`data` = payload bytes after the header.

| Offset    | Type      | Field            | Notes                          |
|-----------|-----------|------------------|--------------------------------|
| [12:16]   | uint32    | status word      | bit flags for validity         |
| [96:100]  | float32   | velocityX (m/s)  | body-forward                   |
| [100:104] | float32   | velocityY (m/s)  | body-lateral (right positive)  |
| [104:108] | float32   | velocityZ (m/s)  | body-vertical (up positive)    |

**Validity bits in the status word:**

| Bit | Name               | Meaning                        |
|-----|--------------------|--------------------------------|
| 6   | beam1FomValid      | Beam 1 FOM (quality) valid     |
| 7   | beam2FomValid      | Beam 2 FOM valid               |
| 9   | xVelocityValid     | X (forward) velocity is valid  |
| 10  | yVelocityValid     | Y (lateral) velocity is valid  |

A velocity sample is only used if `xVelocityValid` (and `yVelocityValid` for lateral
moves) are both set. Mongla also discards steps where `dt > 0.5 s` to guard against
stale packets producing large integration jumps.

---

## 4. AHRS Packet (0xd2) — Heading

`data` = payload bytes after the header.

| Offset                   | Type    | Field    | Notes         |
|--------------------------|---------|----------|---------------|
| [1]                      | uint8   | offsetOfData | variable; marks start of floats |
| [offsetOfData+0 : +4]    | float32 | roll (deg)  |               |
| [offsetOfData+4 : +8]    | float32 | pitch (deg) |               |
| [offsetOfData+8 : +12]   | float32 | heading (deg) | 0–360, NED  |

---

## 5. Unused Packets (future capabilities)

### WaterTrack (0xbe)
Measures velocity relative to the water column instead of the seabed. Useful for:
- Estimating water current direction and speed
- Compensating position integrator for currents in competition pools
- **Not implemented** in `nucleus_parser.py` today. Packet has the same framing as
  BottomTrack; fields are at similar offsets (check Nortek SDK for exact layout).

### Altimeter (0xaa)
Reports distance from DVL transducer to seabed in metres. Useful for:
- Seabed proximity warning (avoid bottoming out)
- Altitude-hold above seabed instead of absolute depth
- **Not implemented** today. Would extend `NucleusDVLSource` with `get_altitude()`.

### ADCP Current Profiling
The Nucleus 1000 supports full ADCP current profiling in depth cells. Requires
Nortek ADCP license. Not relevant for competition pool navigation.

### Dead Reckoning / INS
Hardware dead reckoning using the DVL + internal IMU. Requires Nortek INS license
(separate purchase). Would give absolute XY position in world frame without
accumulating integration drift. Not available in our current hardware config.

---

## 6. TCP Command Sequence

The Nucleus follows a request/response protocol over the same TCP connection that
streams packets. Commands are ASCII strings terminated with `\r\n`.

```
1. TCP connect to 192.168.2.201:9000
2. Wait for banner line (contains firmware version)
3. Send:  SETCLOCKSTR,<ISO8601 timestamp>\r\n   (sync clock; reduces timestamp drift)
4. Send:  GETALL\r\n                             (read all current config)
5. Send:  START\r\n                              (begin streaming packets)
6. Read binary packet stream...
7. Send:  STOP\r\n                               (halt streaming before reconfigure)
```

Authentication step happens implicitly — the password "nortek" is sent as part of
the SETCLOCKSTR flow in `nucleus_dvl.py::connect()`. Check the source for exact byte
sequence.

---

## 7. Position Integrator

`NucleusDVLSource` integrates body-frame velocity to produce `(x_m, y_m)`:

```python
dx = vx * dt   # forward displacement in this step
dy = vy * dt   # lateral displacement

# heading_rad = AHRS heading converted to math convention
x_m += dx * cos(heading_rad) - dy * sin(heading_rad)
y_m += dx * sin(heading_rad) + dy * cos(heading_rad)
```

Call `reset_position()` at the start of each distance move to zero the integrator.
The position is in world frame relative to where `reset_position()` was last called.

`get_position()` returns the current `(x_m, y_m)` tuple under a lock (thread-safe).

---

## 8. POSHOLD Mode (ArduSub + BlueOS Extension)

POSHOLD (ArduSub mode 16) uses EKF3 external nav input from the DVL to hold XY
position. The Nortek BlueOS extension bridges the DVL to ArduSub via
`VISION_POSITION_ESTIMATE` MAVLink messages — no additional Python/ROS2 code is needed.

### Prerequisites

| Item | Requirement |
|------|-------------|
| ArduSub | 4.5.0 recommended (4.1.2 minimum) |
| BlueOS | 1.2.6 or newer |
| Nortek BlueOS extension | Install from BlueOS → Extensions menu |

### ArduSub parameters to set via QGC / MAVProxy

```
EK3_SRC1_POSXY  = 3   (External Nav)
EK3_SRC1_VELXY  = 5   (External Nav)
VISO_TYPE       = 1   (MAVLink)
```

Set, then reboot ArduSub. After reboot verify EKF3 is using DVL:
```bash
# Check EKF3 source status in QGC MAVLink Inspector: ESTIMATOR_STATUS flags
# or watch position hold physically when hovering in POSHOLD
```

### Entering POSHOLD from a mission

```python
# After arming and setting depth, switch to POSHOLD:
duburi.set_mode('POSHOLD')
# AUV holds XY position — use for stationary hold between maneuvers
# or as a more stable base for vision alignment

# Return to ALT_HOLD for distance moves:
duburi.set_mode('ALT_HOLD')
```

POSHOLD will reject the mode change if the DVL is not connected and EKF3 is not
healthy. Always verify DVL connection first with `duburi.dvl_connect()`.

### Pool verification

```bash
# 1. Confirm BlueOS extension is running
#    http://192.168.2.1 → Extensions → Nortek Nucleus → Running

# 2. Confirm MAVLink messages arriving
ros2 run duburi_planner duburi dvl_connect
ros2 topic echo /duburi/state --once | grep mode

# 3. Try POSHOLD manually
ros2 run duburi_planner duburi arm
ros2 run duburi_planner duburi set_depth --target -0.5
ros2 run duburi_planner duburi set_mode --mode POSHOLD
# AUV should hold position; apply gentle lateral push and watch it return
```

---

## 9. DVL in Missions — Current Capabilities

| Capability               | CLI command            | DSL method             |
|--------------------------|------------------------|------------------------|
| Connect DVL              | `dvl_connect`          | *(called at startup)*  |
| Forward DVL move         | `move_forward_dist`    | `move_forward_dist(m)` |
| Backward DVL move        | `move_back_dist`       | `move_back_dist(m)`    |
| Lateral DVL move         | `move_lateral_dist`    | `move_lateral_dist(m)` |
| Position hold mode       | `set_mode POSHOLD`     | `set_mode('POSHOLD')`  |

**Heading lock stays active during all DVL distance moves** — the lock owns Ch4 (yaw
rate) while the DVL drives Ch5 (forward) or Ch6 (lateral). This keeps the AUV
on-heading through long distance moves.

---

## 10. DVL+Vision Display

The vision pipeline already uses OpenCV (`cv2`) end-to-end. To view the annotated
feed alongside AUV state without `rqt_image_view`:

```bash
ros2 run duburi_vision vision_display --ros-args -p camera:=forward
```

This opens a `cv2.imshow()` window with the detector overlay and a HUD showing
depth, yaw, mode, and battery voltage pulled from `/duburi/state`.
Press Q to quit. No Qt/rqt installation required.

---

## 11. Reference: Code Entry Points

| What | Where |
|------|-------|
| TCP connect + auth + START | `nucleus_dvl.py::NucleusDVLSource.connect()` |
| Binary packet decoder | `nucleus_parser.py::parse_packet()` |
| BottomTrack/AHRS decode | `nucleus_parser.py` — `ID_BOTTOMTRACK` / `ID_AHRS` sections |
| Position integrator | `nucleus_dvl.py::_reader_loop()` — `_pos_x / _pos_y` update |
| BNO085 heading + DVL position | `composite_bno_dvl.py::CompositeBnoDvlSource` |
| DVL distance move (forward/back) | `motion_forward.py::drive_forward_dist(signed_dir, ...)` |
| DVL distance move (lateral) | `motion_lateral.py::drive_lateral_dist(...)` |
