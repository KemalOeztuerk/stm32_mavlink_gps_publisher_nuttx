#ifndef MAVLINK_LINK_H
#define MAVLINK_LINK_H

#include "common/mavlink.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAVLINK_LINK_SYSTEM_ID    5U

/* MAV_COMP_ID_AUTOPILOT1 rather than MAV_COMP_ID_GPS (220): a GCS files a
 * component by its ID, and anything that is not the autopilot component is
 * treated as a peripheral hanging off a vehicle rather than as the vehicle
 * itself -- no map icon, no HUD ownership. Paired with MAV_TYPE_GROUND_ROVER
 * in the heartbeat, this board presents as a vehicle in its own right, which
 * is what makes it show up on the map. Nothing downstream depends on the old
 * value: GPS_INPUT is consumed by the receiving autopilot's GPS driver, and
 * FOLLOW_TARGET and the slung-payload script are matched on system ID. */
#define MAVLINK_LINK_COMPONENT_ID 1U /* MAV_COMP_ID_AUTOPILOT1 */

/* Opens the MAVLink UART device (to the autopilot/companion) 8N1 at
 * CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER_MAVLINK_BAUD and keeps the fd for
 * the MAVLink tasks. Call once from main before spawning them. Returns 0 on
 * success, -1 on failure (errno set). */
int MavlinkLink_Init(const char *devpath);

/* Writes one message to the link. Both MAVLink tasks send, so this
 * serialises them; use it instead of touching the fd. */
void MavlinkLink_Send(const mavlink_message_t *msg);

/* Packs and sends one forwarded CAN frame. Packing lives in mavlink_link.c
 * rather than at the call site for the sequence-counter reason explained
 * there. */
void MavlinkLink_SendCanFrame(uint8_t target_system, uint8_t target_component,
                               uint8_t bus, uint8_t len, uint32_t id,
                               const uint8_t *data);

/* pthread entry: at 10Hz sends HEARTBEAT, GPS_INPUT/GPS_RAW_INT from
 * nav_state, HIGHRES_IMU from imu_state, ATTITUDE/VFR_HUD from ahrs_state,
 * FOLLOW_TARGET (so an ArduPilot vehicle in FOLLOW mode can track this
 * board), and GLOBAL_POSITION_INT (the payload feed expected by ArduPilot's
 * copter-slung-payload.lua damping script). */
void *mavlink_tx_task(void *argument);

/* pthread entry: parses the RX line and answers the few messages this board
 * implements -- the CAN bridge that backs Mission Planner's DroneCAN page,
 * and enough of the parameter protocol for Mission Planner to finish
 * connecting. */
void *mavlink_rx_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_LINK_H */
