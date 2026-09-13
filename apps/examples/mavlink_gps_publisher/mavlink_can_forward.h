/* MAVLink <-> raw CAN bridge, so Mission Planner's DroneCAN page can reach
 * the Here4 through this board.
 *
 * Mission Planner offers two ways onto a CAN bus: a direct SLCAN serial
 * adapter, or MAVLink CAN forwarding over an existing link ("MAVLink CAN1",
 * the green button under Initial Setup -> Optional Hardware ->
 * DroneCAN/UAVCAN). This is the second one, implemented to match
 * ArduPilot's AP_MAVLinkCAN so Mission Planner cannot tell the difference:
 *
 *   - MAV_CMD_CAN_FORWARD (param1 = bus + 1, 0 to stop) opens a session and
 *     is re-sent by Mission Planner as a keepalive; the session lapses
 *     CAN_FORWARD_TIMEOUT_MS after the last one.
 *   - Every frame heard on CAN1 is wrapped in a CAN_FRAME message and sent
 *     to whoever opened the session.
 *   - CAN_FRAME messages arriving from Mission Planner are written onto
 *     CAN1 verbatim.
 *   - CAN_FILTER_MODIFY narrows what gets forwarded, which matters a lot on
 *     a slow link (see the bandwidth note in mavlink_can_forward.c).
 *
 * Forwarding is deliberately dumb: Mission Planner is its own DroneCAN node
 * and builds its own transfers, so frames bypass libcanard in both
 * directions and this board is only the wire. Nothing here understands
 * uavcan.protocol.param.* -- it does not have to.
 */
#ifndef MAVLINK_CAN_FORWARD_H
#define MAVLINK_CAN_FORWARD_H

#include <stdbool.h>
#include <stdint.h>

#include "common/mavlink.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t frames_to_gcs;   /* bus -> Mission Planner */
    uint32_t frames_to_bus;   /* Mission Planner -> bus */
    uint32_t drops;           /* frames lost to a full queue, either way */
    uint32_t session_lapses;  /* times the keepalive expired mid-session */
    uint32_t skipped;         /* not forwarded by policy or by the GCS filter */
    uint8_t  filter_ids;      /* size of the current CAN_FILTER_MODIFY list */
} can_forward_stats_t;

/* Registers the CAN receive hook with dronecan_gnss. Call once from main,
 * after DroneCanGnss_Init(). */
void MavlinkCanForward_Init(void);

/* Handles MAV_CMD_CAN_FORWARD from a COMMAND_LONG/COMMAND_INT; sysid/compid
 * are the sender's, which is where forwarded frames get addressed. Returns
 * the MAV_RESULT to acknowledge the command with. */
uint8_t MavlinkCanForward_HandleCommand(float param1, uint8_t sysid,
                                         uint8_t compid);

void MavlinkCanForward_HandleCanFrame(const mavlink_message_t *msg);

void MavlinkCanForward_HandleFilterModify(const mavlink_message_t *msg);

/* Sends queued bus frames out as CAN_FRAME messages, at most a bounded
 * burst per call so the heartbeat still gets out on a slow link. Returns
 * true if frames were still queued when that limit was reached, which tells
 * the caller not to idle the link. */
bool MavlinkCanForward_Drain(void);

/* True while a session is open (and not lapsed). mavlink_tx_task() uses
 * this to stand the periodic telemetry down for the duration. */
bool MavlinkCanForward_Active(void);

/* Runtime master switch, exposed as the CAN_FWD_ENABLE parameter. */
void MavlinkCanForward_SetEnabled(bool enabled);
bool MavlinkCanForward_GetEnabled(void);

/* Whether to keep the full telemetry set running during a session, exposed
 * as the CAN_FWD_TELEM parameter. Off by default: see the bandwidth note. */
void MavlinkCanForward_SetKeepTelemetry(bool keep);
bool MavlinkCanForward_GetKeepTelemetry(void);

/* Whether to forward the Here4's sensor broadcasts too, exposed as the
 * CAN_FWD_BCAST parameter. Off by default -- see frame_is_control_plane()
 * in the .c for why, and turn it on only for Mission Planner's Inspector /
 * Stats views on a link fast enough to carry the whole bus. */
void MavlinkCanForward_SetForwardBroadcasts(bool forward);
bool MavlinkCanForward_GetForwardBroadcasts(void);

void MavlinkCanForward_GetStats(can_forward_stats_t *out);

#ifdef __cplusplus
}
#endif

#endif /* MAVLINK_CAN_FORWARD_H */
