# stm32_mavlink_gps_publisher — NuttX port

NuttX port of [stm32_mavlink_gps_publisher](https://github.com/KemalOeztuerk/stm32_mavlink_gps_publisher):
a custom STM32F105RBT6 board that reads a GPS/compass and an IMU, runs a
complementary-filter AHRS, and streams MAVLink telemetry to a companion
computer or autopilot at 10Hz — HEARTBEAT, GPS_INPUT, GPS_RAW_INT,
GLOBAL_POSITION_INT, HIGHRES_IMU, ATTITUDE, VFR_HUD, and FOLLOW_TARGET (so an
ArduPilot vehicle in FOLLOW mode can track this board). GLOBAL_POSITION_INT
doubles as the payload feed expected by ArduPilot's
[copter-slung-payload.lua](https://ardupilot.org/copter/docs/slung-payload.html)
oscillation-damping script — both it and FOLLOW_TARGET are sent simultaneously,
so this board can serve either use case (or both) without reconfiguration.

It supports two independent, redundant input paths that feed the same
output, each with automatic priority/fallback:

- **GPS position**: a Here4 (DroneCAN, over CAN1) if present, otherwise a
  plain NMEA GPS (over USART2).
- **Attitude (roll/pitch/yaw rates)**: the Here4's onboard IMU (if it
  broadcasts one) if present, otherwise the onboard MPU9250 (over SPI1).
- **Compass heading**: the Here4's magnetometer, tilt-compensated using
  whichever attitude source is active. (The board has no other compass.)

Whichever source is "live" (updated within the last ~2 seconds) wins; if it
goes quiet, the board falls back to the other source automatically. You can
run this with just the onboard NMEA GPS + MPU9250 and no Here4 at all — the
CAN path simply never contributes anything.

## Hardware / pinout

| Peripheral | Pins | Purpose |
|---|---|---|
| USART1 | PA9 (TX) / PA10 (RX), 57600 8N1 | MAVLink out, to your companion computer or autopilot |
| USART2 | PA2 (TX) / PA3 (RX), 9600 8N1 | NMEA GPS in (e.g. an M8N-class module) |
| SPI1 | PA5 (SCK) / PA6 (MISO) / PA7 (MOSI), PA4 (CS) | MPU9250 IMU |
| CAN1 | PA11 (RX) / PA12 (TX), 1Mbit | Here4 (DroneCAN) GNSS/compass/IMU — optional |

There is no debug/NSH console: both onboard UARTs are committed to GPS and
MAVLink, so the firmware boots straight into the application (no shell).
USART3 (PB10/PB11) is unused on this chip if you ever want to wire up a
debug console yourself.

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

The DroneCAN stack (needed for the Here4 path) is downloaded automatically
during the build via curl + unzip (`apps/canutils/libdronecan`). If your
build machine is missing `unzip`, install it first
(`sudo apt-get install unzip`) or the `context` build step will fail with
`unzip: command not found`.

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
MAVLink GPS/IMU publisher example`. Notable ones:

| Option | Default | What it does |
|---|---|---|
| `GPS UART device path` | `/dev/ttyS1` | NMEA GPS serial device |
| `MAVLink UART device path` | `/dev/ttyS0` | MAVLink output serial device |
| `MPU9250 SPI bus number` | `1` | SPI bus the MPU9250 is on |
| `Here4/DroneCAN CAN device path` | `/dev/can0` | CAN device the Here4 is on |
| `DroneCAN local node ID` | `127` | Arbitrary — this node never transmits, just needs to not collide with another node |
| `DroneCAN node memory pool size` | `1024` | Bytes reserved for libcanard's transfer reassembly |

## Flashing

Flash `nuttx.bin` at address `0x08000000` with whatever tool you normally
use for this board — ST-Link Utility / STM32CubeProgrammer, OpenOCD, or
`st-flash write nuttx.bin 0x08000000`.

## Using it

Power up the board. There's no console output to watch (see above), so the
way to verify it's alive is to look at the MAVLink stream on USART1:

- Connect a USB-serial adapter (or your autopilot/companion computer) to
  USART1 at 57600 baud.
- Point a GCS (Mission Planner, QGroundControl) or `mavproxy.py
  --master=/dev/ttyUSBx,57600` at it. You should see a HEARTBEAT
  (`MAV_TYPE_GPS`, system ID 5, component ID 220/`MAV_COMP_ID_GPS`)
  arriving at 10Hz, plus GPS_RAW_INT/GPS_INPUT once the GPS/Here4 gets a fix,
  and HIGHRES_IMU/ATTITUDE once the IMU is running (which is immediately —
  no fix needed).
- If you're feeding this into an ArduPilot vehicle's serial port configured
  for GPS_INPUT, it'll pick up the position from GPS_INPUT; the extra
  messages (GPS_RAW_INT, ATTITUDE, VFR_HUD) are there so you can also see
  the fix on a GCS map/HUD directly from this board during bench testing,
  without a flight controller in the loop.
- For **FOLLOW mode**, point the following vehicle at this board's link —
  it sends FOLLOW_TARGET already.
- For **slung-payload damping** (`copter-slung-payload.lua`), point the
  vehicle's payload-tracking serial port at this board's link — it sends
  GLOBAL_POSITION_INT at 10Hz, which is all that script expects. Both
  FOLLOW_TARGET and GLOBAL_POSITION_INT go out on the same link
  simultaneously, so you don't need to choose one use case over the other.
- If you have a Here4 wired to CAN1, its Fix2/compass/IMU broadcasts take
  over automatically — no configuration needed on this board's side beyond
  it being physically on the bus at 1Mbit. If the Here4's node ID or
  bitrate has been reconfigured away from DroneCAN defaults (e.g. via the
  UAVCAN GUI tool), fix that on the Here4 side, not here.

## Known limitations

- **Compass heading is uncalibrated.** The tilt-compensated heading
  computed from the Here4's magnetometer has no hard-iron/soft-iron offset
  correction — this board never had a compass before, so this is new
  capability, not a regression, but treat the heading as a rough estimate
  until calibration is added.
- **Yaw drifts without a Here4.** With only the onboard MPU9250 (no
  magnetometer on that path), yaw is free-integrated from the gyro alone
  and will drift over time. Roll/pitch are corrected against gravity via
  the accelerometer and don't drift.
- Not tested against real GPS/IMU/Here4 hardware from the environment this
  was developed in — only cross-compiled and verified to link cleanly.
  Flashing and bench-testing is on you.

## Layout / what's actually in this repo

This repo contains just the pieces that don't exist in upstream NuttX —
it's meant to be dropped into checkouts of
[apache/nuttx](https://github.com/apache/nuttx) and
[apache/nuttx-apps](https://github.com/apache/nuttx-apps) as shown above,
not built standalone.

```
nuttx/boards/arm/stm32f1/mavlink-f105/   # board support (clocking, pins, CAN1/SPI1 bring-up)
nuttx/boards-Kconfig.patch               # small additions to boards/Kconfig to register the board
apps/examples/mavlink_gps_publisher/     # the application
```

Inside the application:

| File | Role |
|---|---|
| `mavlink_gps_publisher_main.c` | Entry point; opens devices, starts all threads |
| `nav_state.c` / `imu_state.c` / `ahrs_state.c` | Mutex-protected shared state |
| `gps_nmea.c` | NMEA parser (USART2) |
| `mpu9250.c` | MPU9250 SPI driver |
| `ahrs_filter.c` | Complementary filter shared by the MPU9250 and Here4 IMU paths, with CAN-priority arbitration |
| `dronecan_gnss.c` | Here4/DroneCAN listener (CAN1): GPS fix, compass, IMU |
| `mavlink_link.c` | Builds and sends all MAVLink messages (USART1) |
| `dronecan/generated/` | DroneCAN message (de)serialization, generated by [`DroneCAN/dronecan_dsdlc`](https://github.com/DroneCAN/dronecan_dsdlc) against [`DroneCAN/DSDL`](https://github.com/DroneCAN/DSDL) — not hand-written |
| `mavlink/` | Vendored MAVLink C library (`common`, `standard`, `minimal` dialects) |

## Notable differences from the original firmware

The original was bare-metal STM32 HAL + FreeRTOS with no Here4/CAN support
at all. Besides adding the Here4 path, the port changed:

- **Clock tree**: reaches the same 72MHz SYSCLK/HCLK as the original, but
  routed through PLL2 rather than a direct HSE→PREDIV1 path — NuttX's
  connectivity-line RCC driver always requires PLL2 for this chip family.
- **SPI**: uses NuttX's standard SPI framework (`SPI_LOCK`/`SELECT`/`EXCHANGE`)
  instead of the original's hand-rolled register-level driver.
- **Tasking**: FreeRTOS tasks became pthreads; HAL UART calls became
  `open`/`read`/`write` on `/dev/ttyS0`/`/dev/ttyS1`.
