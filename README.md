# stm32_mavlink_gps_publisher — NuttX port (Here4-only, DroneCAN)

NuttX firmware for a custom STM32F105RBT6 board (ported from
[stm32_mavlink_gps_publisher](https://github.com/KemalOeztuerk/stm32_mavlink_gps_publisher))
that reads **all sensor data from a Here4 over DroneCAN** — GNSS
position/velocity, DOPs/satellite counts, magnetometer, raw IMU, and
barometer — runs a complementary-filter AHRS, and streams MAVLink telemetry
to a companion computer or autopilot at 10Hz: HEARTBEAT, GPS_INPUT,
GPS_RAW_INT, GLOBAL_POSITION_INT, HIGHRES_IMU, SCALED_PRESSURE, ATTITUDE,
VFR_HUD, and FOLLOW_TARGET.

- **FOLLOW mode**: an ArduPilot vehicle in FOLLOW can track this board — it
  sends FOLLOW_TARGET, mirroring ArduPilot's `follow-target-send.lua`.
- **Slung-payload damping**: GLOBAL_POSITION_INT at 10Hz is exactly the
  payload feed expected by ArduPilot's
  [copter-slung-payload.lua](https://ardupilot.org/copter/docs/slung-payload.html).
  Both messages are sent simultaneously, so the board serves either use case
  (or both) without reconfiguration.

The Here4 is the **only** sensor source: the earlier NMEA GPS (M8N) and
MPU9250 fallback paths have been removed entirely, along with the USART2 and
SPI1 peripherals they used.

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
- Our own node ID is static (default 127, configurable) — the allocator
  itself cannot be allocated, per the spec. Assigned IDs stay ≤ 125, so they
  can never collide with it.
- This board also broadcasts its own NodeStatus at 1Hz, as every functioning
  DroneCAN node must.
- A Here4 that was previously given a *static* ID still works — it just skips
  the handshake, and its broadcasts are consumed the same way.

## What is read from the Here4

| DroneCAN message | Feeds |
|---|---|
| `uavcan.equipment.gnss.Fix2` | position, MSL altitude, full NED velocity, fix type (2D/3D/DGPS/RTK-float/RTK-fixed), sats used, PDOP |
| `uavcan.equipment.gnss.Auxiliary` | HDOP, VDOP, sats visible |
| `uavcan.equipment.ahrs.MagneticFieldStrength2` | tilt-compensated compass heading |
| `uavcan.equipment.ahrs.RawIMU` | accel/gyro → complementary-filter AHRS (roll/pitch/yaw + rates) |
| `uavcan.equipment.air_data.StaticPressure` | barometric pressure (HIGHRES_IMU, SCALED_PRESSURE) |
| `uavcan.equipment.air_data.StaticTemperature` | air temperature (HIGHRES_IMU, SCALED_PRESSURE) |
| `uavcan.protocol.NodeStatus` | remote-node bookkeeping for the DNA server |

If the Here4 hardware/firmware doesn't broadcast one of these (e.g. baro or
RawIMU depending on firmware settings), the corresponding MAVLink fields are
simply omitted/flagged unavailable — everything else keeps working.

## Hardware / pinout

| Peripheral | Pins | Purpose |
|---|---|---|
| USART1 | PA9 (TX) / PA10 (RX), 57600 8N1 | MAVLink out, to your companion computer or autopilot |
| CAN1 | PA11 (RX) / PA12 (TX), 1Mbit | Here4 (DroneCAN) — the only sensor source |

There is no debug/NSH console: the firmware boots straight into the
application. USART2 (PA2/PA3) and USART3 (PB10/PB11) are unused if you ever
want to wire up a debug console or a second link.

## Building

```sh
git clone https://github.com/apache/nuttx.git nuttxspace/nuttx
git clone https://github.com/apache/nuttx-apps.git nuttxspace/apps

# Copy this repo's contents in
cp -r mavlink_gps_publisher_nuttx/nuttx/boards/arm/stm32f1/mavlink-f105 \
      nuttxspace/nuttx/boards/arm/stm32f1/
cp -r mavlink_gps_publisher_nuttx/apps/examples/mavlink_gps_publisher \
      nuttxspace/apps/examples/
cd nuttxspace/nuttx
git apply ../../mavlink_gps_publisher_nuttx/nuttx/boards-Kconfig.patch

# Configure and build
./tools/configure.sh mavlink-f105:nsh
make -j$(nproc)
```

This produces `nuttx` (ELF, useful for debugging with symbols) and
`nuttx.bin` (raw binary to flash) in the `nuttx/` directory.

The DroneCAN stack (libcanard) is downloaded automatically during the build
via curl + unzip (`apps/canutils/libdronecan`). If your build machine is
missing `unzip`, install it first (`sudo apt-get install unzip`) or the
`context` build step will fail with `unzip: command not found`.

### Reconfiguring

If you've already configured once and change board settings, force a clean
reconfigure:

```sh
./tools/configure.sh -E mavlink-f105:nsh
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
| `Here4/DroneCAN CAN device path` | `/dev/can0` | CAN device the Here4 is on |
| `DroneCAN local node ID` | `127` | Our own (allocator) node ID; allocated IDs stay ≤ 125 so they never collide |
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
  (`MAV_TYPE_GPS`, system ID 5, component ID 220/`MAV_COMP_ID_GPS`) at 10Hz,
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

- **Compass heading is uncalibrated.** The tilt-compensated heading from the
  Here4's magnetometer has no hard-iron/soft-iron correction — treat it as a
  rough estimate until calibration is added.
- **Yaw drifts.** Yaw is free-integrated from the gyro; the compass feeds the
  reported *heading* fields but is not fused into the AHRS yaw.
- **DNA table is RAM-only** (see above) — fine for a single Here4, but not a
  general-purpose bus allocator with persistence guarantees.
- **One allocation at a time.** Multiple unconfigured allocatees are handled
  by the protocol's random back-off, not by concurrent transactions.
- Not tested against real Here4 hardware from the environment this was
  developed in — cross-compiled and verified to link cleanly (67KB flash /
  51% of the F105RB). Flashing and bench-testing is on you.

## Layout / what's actually in this repo

This repo contains just the pieces that don't exist in upstream NuttX —
it's meant to be dropped into checkouts of
[apache/nuttx](https://github.com/apache/nuttx) and
[apache/nuttx-apps](https://github.com/apache/nuttx-apps) as shown above,
not built standalone.

```
nuttx/boards/arm/stm32f1/mavlink-f105/   # board support (clocking, pins, CAN1 bring-up)
nuttx/boards-Kconfig.patch               # small additions to boards/Kconfig to register the board
apps/examples/mavlink_gps_publisher/     # the application
```

Inside the application:

| File | Role |
|---|---|
| `mavlink_gps_publisher_main.c` | Entry point; opens devices, starts the two threads |
| `nav_state.c` / `imu_state.c` / `ahrs_state.c` / `baro_state.c` | Mutex-protected shared state |
| `ahrs_filter.c` | Complementary filter fed by the Here4's RawIMU |
| `dronecan_gnss.c` | Here4/DroneCAN listener (CAN1) + DNA allocation server + NodeStatus broadcast |
| `mavlink_link.c` | Builds and sends all MAVLink messages (USART1) |
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
