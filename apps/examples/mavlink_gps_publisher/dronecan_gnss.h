/* Here4 (DroneCAN) GNSS + compass listener over CAN1.
 *
 * Passively decodes uavcan.equipment.gnss.Fix2 (position/velocity/fix) into
 * nav_state (NAV_GPS_SOURCE_CAN, which takes priority over the NMEA GPS
 * while it's actively updating -- see nav_state.c) and
 * uavcan.equipment.ahrs.MagneticFieldStrength2 (raw body-frame magnetic
 * field) into a tilt-compensated heading fed to NavState_UpdateHeading().
 *
 * This node never transmits: DroneCAN peripherals broadcast their
 * measurements unconditionally, so no NodeStatus/GetNodeInfo/dynamic node ID
 * allocation handshake is needed for a passive listener.
 */
#ifndef DRONECAN_GNSS_H
#define DRONECAN_GNSS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Opens the CAN device and initializes libcanard. Call once from main
 * before spawning dronecan_task(). Returns 0 on success, -1 on failure. */
int DroneCanGnss_Init(const char *devpath);

/* pthread entry: receives and decodes CAN frames forever. */
void *dronecan_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* DRONECAN_GNSS_H */
