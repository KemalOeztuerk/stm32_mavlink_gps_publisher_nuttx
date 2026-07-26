#ifndef MAVLINK_LINK_H
#define MAVLINK_LINK_H

#ifdef __cplusplus
extern "C" {
#endif

#define MAVLINK_LINK_SYSTEM_ID    5U
#define MAVLINK_LINK_COMPONENT_ID 220U /* MAV_COMP_ID_GPS */

/* Opens the MAVLink UART device (to the autopilot/companion) at 57600 8N1
 * and keeps the fd for mavlink_tx_task() to write to. Call once from main
 * before spawning mavlink_tx_task(). Returns 0 on success, -1 on failure
 * (errno set). */
int MavlinkLink_Init(const char *devpath);

/* pthread entry: at 10Hz sends HEARTBEAT, GPS_INPUT/GPS_RAW_INT from
 * nav_state, HIGHRES_IMU from imu_state, ATTITUDE/VFR_HUD from ahrs_state,
 * FOLLOW_TARGET (so an ArduPilot vehicle in FOLLOW mode can track this
 * board), and GLOBAL_POSITION_INT (the payload feed expected by ArduPilot's
 * copter-slung-payload.lua damping script). */
void *mavlink_tx_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_LINK_H */
