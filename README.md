# stm32_mavlink_gps_publisher — NuttX port (Here4-only, DroneCAN)

NuttX firmware for a custom STM32F105RBT6 or STM32F446RET6 board (ported from
[stm32_mavlink_gps_publisher](https://github.com/KemalOeztuerk/stm32_mavlink_gps_publisher))
that reads **all sensor data from a Here4 over DroneCAN** — GNSS
position/velocity, DOPs/satellite counts, magnetometer, raw IMU, and
barometer — runs a complementary-filter AHRS, and streams MAVLink telemetry
to a companion computer or autopilot at 10Hz: HEARTBEAT, GPS_INPUT,
GPS_RAW_INT, GLOBAL_POSITION_INT, HIGHRES_IMU, SCALED_PRESSURE, ATTITUDE,
VFR_HUD, and FOLLOW_TARGET.

- **FOLLOW mode**: an ArduPilot vehicle in FOLLOW can track this board — it
  sends FOLLOW_TARGET, mirroring ArduPilot's `follow-target-send.lua`.
- **Slung-payload damping**: GLOBAL_POSITION_INT carries true MSL altitude
  in `alt`, which is the field that script reads; `relative_alt` is height
  gained since the first 3D fix, because this board has no home position.
  At 10Hz it is exactly the payload feed expected by ArduPilot's
  [copter-slung-payload.lua](https://ardupilot.org/copter/docs/slung-payload.html).
  Both messages are sent simultaneously, so the board serves either use case
  (or both) without reconfiguration.

- **Here4 configuration from Mission Planner**: the MAVLink link doubles as
  a CAN bridge, so Mission Planner's DroneCAN page reaches the Here4 through
  this board -- no separate CAN adapter. See below.

The Here4 is the **only** sensor source: the earlier NMEA GPS (M8N) and
MPU9250 fallback paths have been removed entirely, along with the USART2 and
SPI1 peripherals they used.

## Supported boards

Two board ports are included. They share the application, the pinout and the
Here4 wiring, and differ only in the MCU fitted:

| Config | MCU | SYSCLK | Flash / RAM | Firmware size |
|---|---|---|---|---|
| `mavlink-f105:nsh` | STM32F105RBT6 (Cortex-M3, soft float) | 72MHz | 128K / 64K | 77K (59% of flash) |
| `mavlink-f446:nsh` | STM32F446RET6 (Cortex-M4F, hardware FPU) | 180MHz | 512K / 128K | 79K (15% of flash) |

Both assume the same 8MHz HSE crystal and put the same two peripherals on the
same package pins, so one PCB layout can be populated with either part. The
F446 build enables the hardware FPU, so the AHRS complementary filter and the
tilt-compensated compass maths run in hardware rather than in libgcc
soft-float, and it leaves far more room to grow into.

## Dynamic node allocation — no Here4 configuration needed

This board runs the **centralized dynamic-node-ID allocation (DNA) server**
from the DroneCAN spec. A factory-fresh Here4 boots without a node ID and
requests one anonymously; this board collects its 16-byte unique ID over the
three-stage handshake and assigns it an ID (from 125 downward, or the Here4's
preferred ID if free). You never have to set a static node ID on the Here4 —
plug it into CAN1 at 1Mbit and it comes up on its own.

Details worth knowing:

- The allocation table is RAM-only. If this board reboots, the Here4 simply
  re-requests and is re-assigned (the same ID, deterministically, for as long
  as this board stays up thereafter). No flash storage is involved.
- IDs already heard on the bus (any node's traffic) are never handed out.
- Our own node ID is static (default 126, configurable) — the allocator
  itself cannot be allocated, per the spec. Assigned IDs stay ≤ 125, so they
  can never collide with it. 126 rather than 127 because Mission Planner
  joins the bus as node 127 when you open its DroneCAN page through this
  board (see below), and two nodes sharing an ID breaks service transfers.
- This board also broadcasts its own NodeStatus at 1Hz, as every functioning
  DroneCAN node must.
- A Here4 that was previously given a *static* ID still works — it just skips
  the handshake, and its broadcasts are consumed the same way.

## What is read from the Here4

| DroneCAN message | Feeds |
|---|---|
| `uavcan.equipment.gnss.Fix2` | position, MSL altitude, full NED velocity, fix type (2D/3D/DGPS/RTK-float/RTK-fixed), sats used, PDOP |
| `uavcan.equipment.gnss.Auxiliary` | HDOP, VDOP, sats visible |
| `uavcan.equipment.ahrs.MagneticFieldStrength2` | tilt-compensated compass heading (also drives VFR_HUD's heading when there is no AHRS) |
| `uavcan.equipment.ahrs.RawIMU` | accel/gyro → complementary-filter AHRS (roll/pitch/yaw + rates) |
| `uavcan.equipment.air_data.StaticPressure` | barometric pressure (HIGHRES_IMU, SCALED_PRESSURE) |
| `uavcan.equipment.air_data.StaticTemperature` | air temperature (HIGHRES_IMU, SCALED_PRESSURE) |
| `uavcan.protocol.NodeStatus` | remote-node bookkeeping for the DNA server |

If the Here4 hardware/firmware doesn't broadcast one of these (e.g. baro or
RawIMU depending on firmware settings), the corresponding MAVLink fields are
simply omitted/flagged unavailable — everything else keeps working.

## Configuring the Here4 from Mission Planner

The board forwards raw CAN frames over the MAVLink link, the same way an
ArduPilot autopilot does (`MAV_CMD_CAN_FORWARD` plus `CAN_FRAME`, matching
`AP_MAVLinkCAN`), so Mission Planner can browse and edit the Here4's own
DroneCAN parameters through it:

1. Connect Mission Planner to the board's MAVLink link as usual.
2. **Initial Setup -> Optional Hardware -> DroneCAN/UAVCAN**.
3. Press the green **MAVLink CAN1** button (not the red SLCAN one -- that
   path wants a CAN adapter on its own COM port).
4. The Here4 appears in the node list; **Parameters** opens its settings,
   and **Menu -> Update** can flash its firmware.

Mission Planner keeps the session alive by re-sending the command; five
seconds after it stops (leaving the page, disconnecting) the board goes back
to normal.

The forwarding is deliberately dumb: Mission Planner runs its own DroneCAN
stack and this board is only the wire, so frames bypass libcanard in both
directions. Nothing in this firmware has to understand
`uavcan.protocol.param.*`, which is why editing Here4 parameters works
without any per-parameter support here.

### What gets forwarded, and why not everything

A 57600 link carries roughly 200 CAN frames per second once each one is
wrapped in a MAVLink `CAN_FRAME`. The Here4 alone puts more than that on
CAN1 -- Fix2 is a multi-frame transfer, and the magnetometer, raw IMU and
barometer broadcasts pile on top. Forwarding all of it does not just waste
the link, it queues ahead of the service responses Mission Planner is
waiting on, so its parameter enumeration stops at a different random index
on every refresh.

So the bridge forwards only what a DroneCAN page actually needs:

- service transfers (`GetNodeInfo`, `param.GetSet`, `RestartNode`, and the
  file reads behind a firmware update),
- `NodeStatus`, so nodes appear in the list at all,
- anonymous frames, which is how node-ID allocation happens.

The Here4's sensor broadcasts are dropped -- this firmware is already
consuming them locally, and Mission Planner has no use for them. Set
`CAN_FWD_BCAST=1` to forward the whole bus instead, which is what Mission
Planner's **Inspector** and **Stats** views want; expect it to need a faster
link than 57600.

On top of that, the periodic telemetry stands down for the duration of a
session -- only HEARTBEAT keeps going, so Mission Planner stays connected --
and is replaced by a 1Hz diagnostic STATUSTEXT you can watch in the Messages
tab. Everything resumes by itself when the session lapses.

The line reads `CFWD e0 s0 t0 d0 k11375 f0 o575 i194`, and its fields are
ordered so the ones that localise a fault come first:

| Field | Meaning |
|---|---|
| `e` | MAVLink receive errors. Non-zero means bytes are being lost between the GCS and this board, not in the forwarding. |
| `s` | Times the session lapsed on its 5s keepalive. Any lapse mid-download silently discards responses. |
| `t` | CAN transmit errors: requests that never reached the bus. |
| `d` | Frames dropped because the forward queue was full. This is the bandwidth symptom. |
| `k` | Frames deliberately not forwarded (sensor broadcasts, plus anything `CAN_FILTER_MODIFY` excluded). Counted so the policy can never be mistaken for loss. |
| `f` | How many IDs the GCS filtered down to. |
| `o` / `i` | Frames out to the GCS / in to the bus, for scale. |

For *firmware updates* over this link, raise `MAVLink UART baud rate` (see
below) -- 921600 is a good choice -- and change the other end to match.

### Runtime parameters

Mission Planner downloads a parameter list as soon as it connects, so the
board now answers the parameter protocol with two entries. Neither is
persisted (this board has no storage), so both revert on reboot:

| Parameter | Default | What it does |
|---|---|---|
| `CAN_FWD_ENABLE` | 1 | Master switch for CAN forwarding. Set to 0 to refuse sessions entirely. |
| `CAN_FWD_TELEM` | 0 | Set to 1 to keep the full telemetry stream running during a session. Only useful on a link with bandwidth to spare. |
| `CAN_FWD_BCAST` | 0 | Set to 1 to forward the Here4's sensor broadcasts as well, for Mission Planner's Inspector/Stats views. Costs the bandwidth the service traffic needs. |

## Hardware / pinout

| Peripheral | Pins | Purpose |
|---|---|---|
| USART1 | PA9 (TX) / PA10 (RX), 57600 8N1 | MAVLink out, to your companion computer or autopilot |
| CAN1 | PA11 (RX) / PA12 (TX), 1Mbit | Here4 (DroneCAN) — the only sensor source |

The pinout is identical on both MCUs: the STM32F105RB and the STM32F446RE
bring USART1 and CAN1 out on the same package pins (AF7 and AF9 respectively
on the F446, no remap on the F105), so the same board works with either part
fitted.

There is no debug/NSH console: the firmware boots straight into the
application. USART2 (PA2/PA3) and USART3 (PB10/PB11) are unused if you ever
want to wire up a debug console or a second link.

## Building

```sh
git clone https://github.com/apache/nuttx.git nuttxspace/nuttx
git clone https://github.com/apache/nuttx-apps.git nuttxspace/apps

# Copy this repo's contents in (both boards, or just the one you need)
cp -r mavlink_gps_publisher_nuttx/nuttx/boards/arm/stm32f1/mavlink-f105 \
      nuttxspace/nuttx/boards/arm/stm32f1/
cp -r mavlink_gps_publisher_nuttx/nuttx/boards/arm/stm32f4/mavlink-f446 \
      nuttxspace/nuttx/boards/arm/stm32f4/
cp -r mavlink_gps_publisher_nuttx/apps/examples/mavlink_gps_publisher \
      nuttxspace/apps/examples/
cd nuttxspace/nuttx
git apply ../../mavlink_gps_publisher_nuttx/nuttx/boards-Kconfig.patch

# Configure and build -- pick the board matching the MCU you fitted
./tools/configure.sh mavlink-f105:nsh    # STM32F105RBT6
# ./tools/configure.sh mavlink-f446:nsh  # STM32F446RET6
make -j$(nproc)
```

This produces `nuttx` (ELF, useful for debugging with symbols) and
`nuttx.bin` (raw binary to flash) in the `nuttx/` directory.

The DroneCAN stack (libcanard) is downloaded automatically during the build
via curl + unzip (`apps/canutils/libdronecan`). If your build machine is
missing `unzip`, install it first (`sudo apt-get install unzip`) or the
`context` build step will fail with `unzip: command not found`.

### Reconfiguring

If you've already configured once and change board settings -- or you are
switching between the two boards -- force a clean reconfigure:

```sh
./tools/configure.sh -E mavlink-f105:nsh
./tools/configure.sh -E mavlink-f446:nsh
```

### Changing settings (device paths, node ID, stack sizes, etc.)

```sh
make menuconfig
```
Application options are under `Application Configuration → Examples →
MAVLink GPS/IMU publisher example`:

| Option | Default | What it does |
|---|---|---|
| `MAVLink UART device path` | `/dev/ttyS0` | MAVLink output serial device |
| `MAVLink UART baud rate` | `57600` | Link speed. Raise it (e.g. 921600) for DroneCAN firmware updates through Mission Planner |
| `Here4/DroneCAN CAN device path` | `/dev/can0` | CAN device the Here4 is on |
| `DroneCAN local node ID` | `126` | Our own (allocator) node ID; allocated IDs stay ≤ 125, and Mission Planner uses 127 |
| `DroneCAN node memory pool size` | `1024` | Bytes for libcanard's transfer reassembly + TX queue |

## Flashing

Flash `nuttx.bin` at address `0x08000000` with whatever tool you normally
use for this board — ST-Link Utility / STM32CubeProgrammer, OpenOCD, or
`st-flash write nuttx.bin 0x08000000`.

## Using it

Power up the board with the Here4 on CAN1. There's no console output, so
verify life by watching the MAVLink stream on USART1:

- Connect a USB-serial adapter (or your autopilot/companion computer) to
  USART1 at 57600 baud.
- Point a GCS (Mission Planner, QGroundControl) or `mavproxy.py
  --master=/dev/ttyUSBx,57600` at it. You should see a HEARTBEAT
  (`MAV_TYPE_GROUND_ROVER`, system ID 5, component ID 1) at 10Hz,
  ATTITUDE/HIGHRES_IMU as soon as the Here4's IMU broadcasts arrive, and
  GPS_RAW_INT/GPS_INPUT/GLOBAL_POSITION_INT once it has a fix.
- The Here4's LED should go from its "no node ID" state to normal operation
  within a couple of seconds of both being powered — that's the DNA handshake
  completing.
- If you're feeding an ArduPilot vehicle's serial port configured for
  GPS_INPUT, it picks up the position from GPS_INPUT; the extra messages
  (GPS_RAW_INT, ATTITUDE, VFR_HUD, SCALED_PRESSURE) let you watch the fix on
  a GCS map/HUD directly during bench testing.
- For **FOLLOW mode**, point the following vehicle at this board's link — it
  sends FOLLOW_TARGET already.
- For **slung-payload damping** (`copter-slung-payload.lua`), point the
  vehicle's payload-tracking serial port at this board's link — it sends
  GLOBAL_POSITION_INT at 10Hz.

## Known limitations

- **The Here4 may not broadcast RawIMU at all.** On the unit tested here
  both `rate_gyro_latest` and `rate_gyro_integral` come through as zeroes,
  so the AHRS never initialises and ATTITUDE/HIGHRES_IMU carry nothing. The
  cause is on the peripheral: AP_Periph only starts its IMU thread when
  `IMU_SAMPLE_RATE` is non-zero, and a Here4 ships with it at 0. Set it to
  50 through the DroneCAN page above and reboot the Here4 -- 50Hz because
  `ahrs_filter.c` assumes a 0.02s nominal period, and its complementary
  filter constant is tuned against that. The `RAW gl:.. gi:..` STATUSTEXT
  exists to make the whole thing visible.
- **Compass calibration is hard-iron only, and per-boot.** A running
  min/max calibration learns the hard-iron offset as the unit is rotated,
  and announces itself on the MAVLink link (`MAGCAL ofs .. (0.01Ga)`) once
  the horizontal axes have seen enough span -- a slow full turn after
  power-up completes it. Until then the heading is served uncalibrated and
  is not allowed to steer the AHRS yaw. There is no soft-iron correction,
  and nothing is stored, so the turn is needed again after every reboot.
- **Yaw is gyro-integrated, anchored to the compass.** The first calibrated
  heading snaps yaw to it; after that the compass corrects yaw slowly
  whenever a fresh heading is available. With no magnetometer fix for a
  while, yaw drifts at the gyro's bias rate.
- **DNA table is RAM-only** (see above) — fine for a single Here4, but not a
  general-purpose bus allocator with persistence guarantees.
- **One allocation at a time.** Multiple unconfigured allocatees are handled
  by the protocol's random back-off, not by concurrent transactions.
- The Mission Planner CAN bridge has been driven far enough to read a real
  Here4's node info and parameters over it, but not to run a DroneCAN
  firmware update. If Mission Planner does not list the Here4, check that
  the green *MAVLink CAN1* button was used (not the red SLCAN one) and that
  the `CFWD` STATUSTEXT shows frames moving. Parameter lists that come back
  a different length on every refresh, with every `CFWD` counter still at
  zero, are a round-trip-latency problem rather than a loss one -- raising
  the link baud is the lever that helps.
- The STM32F446RE port has been run on hardware: it boots, USART1 carries
  MAVLink to a GCS, CAN1 runs at 1Mbit against a real Here4, and the Here4
  (which ships with `CAN_NODE=0`, no static ID) gets its node ID from the
  DNA server here. That exercises the 180MHz over-drive clock tree and the
  45MHz-PCLK1 CAN bit timing worked out in
  `nuttx/boards/arm/stm32f4/mavlink-f446/include/board.h`. The GNSS path has
  since been seen working on it too -- a 3D fix decoded from Fix2, and the
  magnetometer's running hard-iron calibration completing and reporting its
  offsets. The AHRS is still unconfirmed, but for a reason that has nothing
  to do with this board: the Here4 on hand broadcasts RawIMU as all zeroes
  (see the known limitation below).
- The STM32F105RB build is cross-compiled and verified to link cleanly
  (77KB flash, 59% of the part; the F446RE build is 79KB, 15%), but the
  hardware time since the DroneCAN work started has all been on the F446.

## Layout / what's actually in this repo

This repo contains just the pieces that don't exist in upstream NuttX —
it's meant to be dropped into checkouts of
[apache/nuttx](https://github.com/apache/nuttx) and
[apache/nuttx-apps](https://github.com/apache/nuttx-apps) as shown above,
not built standalone.

```
nuttx/boards/arm/stm32f1/mavlink-f105/   # STM32F105RB board support (clocking, pins, CAN1 bring-up)
nuttx/boards/arm/stm32f4/mavlink-f446/   # STM32F446RE board support, same pinout
nuttx/boards-Kconfig.patch               # small additions to boards/Kconfig to register both boards
apps/examples/mavlink_gps_publisher/     # the application (identical for both boards)
```

Inside the application:

| File | Role |
|---|---|
| `mavlink_gps_publisher_main.c` | Entry point; opens devices, starts the two threads |
| `nav_state.c` / `imu_state.c` / `ahrs_state.c` / `baro_state.c` | Mutex-protected shared state |
| `ahrs_filter.c` | Complementary filter fed by the Here4's RawIMU |
| `dronecan_gnss.c` | Here4/DroneCAN listener (CAN1) + DNA allocation server + NodeStatus broadcast |
| `mavlink_link.c` | Builds and sends all MAVLink messages (USART1); parses the RX line for commands and parameters |
| `mavlink_can_forward.c` | MAVLink/CAN bridge behind Mission Planner's DroneCAN page |
| `dronecan/generated/` | DroneCAN message (de)serialization, generated by [`DroneCAN/dronecan_dsdlc`](https://github.com/DroneCAN/dronecan_dsdlc) against [`DroneCAN/DSDL`](https://github.com/DroneCAN/DSDL) — not hand-written |
| `mavlink/` | Vendored MAVLink C library (`common`, `standard`, `minimal` dialects) |

## History

The original firmware was bare-metal STM32 HAL + FreeRTOS reading an NMEA
GPS and an MPU9250, with no CAN support. The first NuttX port kept those as
fallbacks alongside the Here4. This version drops them entirely: the Here4
is the single source for GNSS, compass, IMU, and baro, and the board now
participates actively on the bus (NodeStatus + DNA server) instead of being
a passive listener with a hardcoded assumption that the Here4 already has a
node ID.
