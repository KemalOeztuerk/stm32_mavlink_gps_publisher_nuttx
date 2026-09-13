#include "mavlink_can_forward.h"
#include "mavlink_link.h"
#include "dronecan_gnss.h"
#include "clock_ms.h"

#include "uavcan.protocol.NodeStatus.h"

#include <pthread.h>
#include <string.h>

/* Mission Planner holds a forwarding session open by re-sending
 * MAV_CMD_CAN_FORWARD; ArduPilot drops the session 5s after the last one
 * and so do we, so closing the DroneCAN page stops the flood by itself. */
#define CAN_FORWARD_TIMEOUT_MS 5000U

/* Frames captured off the bus between drains. A Here4 can put several
 * hundred frames a second on CAN1 (a Fix2 alone is a multi-frame transfer),
 * so this only has to ride out one tick's worth plus a service exchange. */
#define CAN_FORWARD_QUEUE_LEN 128

/* Bandwidth, the thing that actually limits this feature: a CAN_FRAME is 16
 * payload bytes, about 28 on the wire with MAVLink v2 framing. At the
 * default 57600 that caps out near 200 frames/s -- below what a Here4 emits
 * -- which is why mavlink_tx_task() stands the periodic telemetry down for
 * the duration of a session, and why Mission Planner's CAN_FILTER_MODIFY is
 * honoured. If you want firmware updates over this link rather than just
 * parameter edits, raise CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER_MAVLINK_BAUD. */
#define CAN_FORWARD_DRAIN_PER_TICK 16

/* Mission Planner sends at most 16 IDs per CAN_FILTER_MODIFY but can build
 * a longer list with repeated ADD operations. */
#define CAN_FORWARD_MAX_FILTER_IDS 48

/* On a slow link the Here4 outruns the queue by a wide margin, and the
 * frames Mission Planner is actually waiting on -- the service transfers
 * carrying GetNodeInfo and param.GetSet -- are a handful among hundreds of
 * sensor broadcasts. Dropping indiscriminately loses them in that noise,
 * which shows up as a node listed with an empty name and an empty parameter
 * table. So past this depth only service frames get in: a dropped broadcast
 * is repeated a few milliseconds later anyway, while a dropped service
 * response costs a visible retry or a failed page. */
#define CAN_FORWARD_SERVICE_ONLY_DEPTH (CAN_FORWARD_QUEUE_LEN / 2)

struct fwd_frame {
    uint32_t id;
    uint8_t len;
    uint8_t data[8];
};

static pthread_mutex_t s_mutex;

static struct fwd_frame s_queue[CAN_FORWARD_QUEUE_LEN];
static uint16_t s_head;
static uint16_t s_tail;

static bool s_enabled = true;
static bool s_keep_telemetry;
static bool s_forward_broadcasts;
static bool s_session;
static uint8_t s_target_sysid;
static uint8_t s_target_compid;
static uint32_t s_last_enable_ms;

static uint16_t s_filter_ids[CAN_FORWARD_MAX_FILTER_IDS];
static uint8_t s_num_filter_ids;

static can_forward_stats_t s_stats;

/* Caller holds the mutex. */
static bool session_live(void)
{
    if (!s_session) {
        return false;
    }
    if ((clock_now_ms() - s_last_enable_ms) > CAN_FORWARD_TIMEOUT_MS) {
        /* Counted: a lapse in the middle of a parameter download drops
         * every response until Mission Planner's next keepalive, which
         * would look exactly like frame loss. */
        s_session = false;
        s_num_filter_ids = 0;
        s_head = s_tail = 0;
        s_stats.session_lapses++;
        return false;
    }
    return true;
}

/* The ID Mission Planner filters on is the DroneCAN type ID carved out of
 * the 29-bit CAN ID, not the CAN ID itself. Bit 7 is service-not-message
 * and bits 0..6 are the source node ID, so an all-zero low byte means an
 * anonymous message frame (type ID 0). Lifted from ArduPilot's
 * AP_MAVLinkCAN so the same filter list means the same thing here.
 *
 * Caller holds the mutex. An empty list passes everything. */
static bool filter_allows(uint32_t can_id)
{
    if (s_num_filter_ids == 0) {
        return true;
    }

    uint16_t id = 0;
    if ((can_id & 0xff) != 0) {
        if (can_id & 0x80) {
            id = (uint16_t)(uint8_t)(can_id >> 16);  /* service frame */
        } else {
            id = (uint16_t)(can_id >> 8);            /* message frame */
        }
    }

    /* The list is kept sorted, so stop as soon as we pass the target. */
    for (uint8_t i = 0; i < s_num_filter_ids; i++) {
        if (s_filter_ids[i] == id) {
            return true;
        }
        if (s_filter_ids[i] > id) {
            break;
        }
    }
    return false;
}

/* Caller holds the mutex. */
static uint16_t queue_depth(void)
{
    return (uint16_t)((s_head + CAN_FORWARD_QUEUE_LEN - s_tail) %
                      CAN_FORWARD_QUEUE_LEN);
}

/* What a GCS on the other end of this bridge actually needs: the service
 * transfers it makes (GetNodeInfo, param.GetSet, RestartNode, the file reads
 * behind a firmware update), NodeStatus so nodes appear in its list at all,
 * and the anonymous traffic of node-ID allocation.
 *
 * Everything else on CAN1 is the Here4's sensor feed -- Fix2, RawIMU,
 * magnetometer, barometer -- which this firmware consumes locally and which
 * Mission Planner has no use for on its DroneCAN page. Forwarding it is not
 * merely wasteful: those broadcasts alone exceed what a 57600 link can
 * carry, so they queue ahead of the service responses and either crowd them
 * out or delay them past the GCS's timeout. The visible symptom is a
 * parameter list that stops at a different random index on every refresh.
 *
 * CAN_FWD_BCAST=1 forwards the whole bus instead, for Mission Planner's
 * Inspector and Stats views -- worth having, but only on a link with the
 * bandwidth for it. */
static bool frame_is_control_plane(uint32_t can_id)
{
    if ((can_id & 0x80) != 0) {
        return true;  /* service request or response */
    }
    if ((can_id & 0x7f) == 0) {
        return true;  /* anonymous: node ID allocation */
    }
    return (uint16_t)(can_id >> 8) == UAVCAN_PROTOCOL_NODESTATUS_ID;
}

/* Runs in dronecan_task context for every frame off the bus, so it only
 * touches the queue -- the UART write happens later, in the drain. */
static void can_rx_hook(uint32_t can_id, uint8_t len, const uint8_t *data)
{
    if (len > 8) {
        len = 8;
    }

    pthread_mutex_lock(&s_mutex);

    if (!s_enabled || !session_live()) {
        pthread_mutex_unlock(&s_mutex);
        return;
    }

    if (!filter_allows(can_id)) {
        /* Counted: a filter that is subtly wrong discards frames the GCS
         * was waiting for, and without this it looks identical to a frame
         * that was never on the bus. */
        s_stats.skipped++;
        pthread_mutex_unlock(&s_mutex);
        return;
    }

    if (!s_forward_broadcasts && !frame_is_control_plane(can_id)) {
        /* Deliberately not forwarded: counted separately from drops so the
         * policy stays visible and never looks like loss. */
        s_stats.skipped++;
        pthread_mutex_unlock(&s_mutex);
        return;
    }

    /* Bit 7 of the CAN ID is service-not-message. */
    bool is_service = (can_id & 0x80) != 0;
    if (!is_service && queue_depth() >= CAN_FORWARD_SERVICE_ONLY_DEPTH) {
        s_stats.drops++;
        pthread_mutex_unlock(&s_mutex);
        return;
    }

    uint16_t next = (uint16_t)((s_head + 1) % CAN_FORWARD_QUEUE_LEN);
    if (next == s_tail) {
        /* Full: drop the new frame rather than overwrite one the drain may
         * already be reading. DroneCAN service calls are retried by the
         * caller, so a lost frame costs a retry, not a wedged session. */
        s_stats.drops++;
        pthread_mutex_unlock(&s_mutex);
        return;
    }

    s_queue[s_head].id = can_id;
    s_queue[s_head].len = len;
    memcpy(s_queue[s_head].data, data, len);
    s_head = next;

    pthread_mutex_unlock(&s_mutex);
}

void MavlinkCanForward_Init(void)
{
    pthread_mutex_init(&s_mutex, NULL);
    s_head = s_tail = 0;
    s_session = false;
    s_num_filter_ids = 0;
    s_enabled = true;
    s_keep_telemetry = false;
    s_forward_broadcasts = false;
    memset(&s_stats, 0, sizeof(s_stats));

    DroneCanGnss_SetRxHook(can_rx_hook);
}

uint8_t MavlinkCanForward_HandleCommand(float param1, uint8_t sysid,
                                         uint8_t compid)
{
    /* ArduPilot's convention: param1 is the 1-based bus number, so bus =
     * param1 - 1 and param1 == 0 (bus -1) means stop. This board has one
     * bus, CAN1. */
    int bus = (int)(int8_t)param1 - 1;

    pthread_mutex_lock(&s_mutex);

    if (bus < 0) {
        s_session = false;
        s_num_filter_ids = 0;
        s_head = s_tail = 0;
        pthread_mutex_unlock(&s_mutex);
        return MAV_RESULT_ACCEPTED;
    }

    if (bus != 0) {
        pthread_mutex_unlock(&s_mutex);
        return MAV_RESULT_DENIED;
    }

    if (!s_enabled) {
        pthread_mutex_unlock(&s_mutex);
        return MAV_RESULT_TEMPORARILY_REJECTED;
    }

    if (!s_session) {
        /* Fresh session: start from an empty queue so Mission Planner is
         * not handed frames from before it asked. */
        s_head = s_tail = 0;
        s_num_filter_ids = 0;
        s_session = true;
    }
    s_target_sysid = sysid;
    s_target_compid = compid;
    s_last_enable_ms = clock_now_ms();

    pthread_mutex_unlock(&s_mutex);
    return MAV_RESULT_ACCEPTED;
}

void MavlinkCanForward_HandleCanFrame(const mavlink_message_t *msg)
{
    mavlink_can_frame_t frame;
    mavlink_msg_can_frame_decode(msg, &frame);

    if (frame.bus != 0 || frame.len > 8) {
        return;
    }

    pthread_mutex_lock(&s_mutex);
    bool live = s_enabled && session_live();
    pthread_mutex_unlock(&s_mutex);
    if (!live) {
        return;
    }

    /* libcanard and ArduPilot agree on the flag layout in the top three
     * bits (EFF in bit 31), so the ID goes onto the bus untouched. */
    bool queued = DroneCanGnss_QueueTxFrame(frame.id, frame.len, frame.data);

    pthread_mutex_lock(&s_mutex);
    if (queued) {
        s_stats.frames_to_bus++;
    } else {
        s_stats.drops++;
    }
    pthread_mutex_unlock(&s_mutex);
}

/* Caller holds the mutex. Keeps the list sorted and duplicate-free so
 * filter_allows() can stop early. */
static void filter_insert(uint16_t id)
{
    uint8_t i;
    for (i = 0; i < s_num_filter_ids; i++) {
        if (s_filter_ids[i] == id) {
            return;
        }
        if (s_filter_ids[i] > id) {
            break;
        }
    }
    if (s_num_filter_ids >= CAN_FORWARD_MAX_FILTER_IDS) {
        return;
    }
    memmove(&s_filter_ids[i + 1], &s_filter_ids[i],
            (size_t)(s_num_filter_ids - i) * sizeof(s_filter_ids[0]));
    s_filter_ids[i] = id;
    s_num_filter_ids++;
}

/* Caller holds the mutex. */
static void filter_remove(uint16_t id)
{
    for (uint8_t i = 0; i < s_num_filter_ids; i++) {
        if (s_filter_ids[i] == id) {
            memmove(&s_filter_ids[i], &s_filter_ids[i + 1],
                    (size_t)(s_num_filter_ids - i - 1) * sizeof(s_filter_ids[0]));
            s_num_filter_ids--;
            return;
        }
    }
}

void MavlinkCanForward_HandleFilterModify(const mavlink_message_t *msg)
{
    mavlink_can_filter_modify_t mod;
    mavlink_msg_can_filter_modify_decode(msg, &mod);

    if (mod.bus != 0) {
        return;
    }

    uint8_t num_ids = mod.num_ids;
    if (num_ids > 16) {
        num_ids = 16; /* the message carries no more than this */
    }

    pthread_mutex_lock(&s_mutex);

    if (mod.operation == CAN_FILTER_REPLACE) {
        s_num_filter_ids = 0;
    }

    if (mod.operation == CAN_FILTER_REMOVE) {
        for (uint8_t i = 0; i < num_ids; i++) {
            filter_remove(mod.ids[i]);
        }
    } else {
        for (uint8_t i = 0; i < num_ids; i++) {
            filter_insert(mod.ids[i]);
        }
    }

    s_stats.filter_ids = s_num_filter_ids;
    pthread_mutex_unlock(&s_mutex);
}

bool MavlinkCanForward_Drain(void)
{
    for (unsigned n = 0; n < CAN_FORWARD_DRAIN_PER_TICK; n++) {
        struct fwd_frame frame;
        uint8_t sysid;
        uint8_t compid;

        pthread_mutex_lock(&s_mutex);
        if (s_tail == s_head) {
            pthread_mutex_unlock(&s_mutex);
            return false;
        }
        frame = s_queue[s_tail];
        s_tail = (uint16_t)((s_tail + 1) % CAN_FORWARD_QUEUE_LEN);
        sysid = s_target_sysid;
        compid = s_target_compid;
        s_stats.frames_to_gcs++;
        pthread_mutex_unlock(&s_mutex);

        /* Sent outside the lock: the UART write blocks on a slow link and
         * the hook has to stay free to keep filling the queue. */
        MavlinkLink_SendCanFrame(sysid, compid, 0 /* bus */,
                                  frame.len, frame.id, frame.data);
    }

    pthread_mutex_lock(&s_mutex);
    bool backlog = s_tail != s_head;
    pthread_mutex_unlock(&s_mutex);
    return backlog;
}

bool MavlinkCanForward_Active(void)
{
    pthread_mutex_lock(&s_mutex);
    bool live = s_enabled && session_live();
    pthread_mutex_unlock(&s_mutex);
    return live;
}

void MavlinkCanForward_SetEnabled(bool enabled)
{
    pthread_mutex_lock(&s_mutex);
    s_enabled = enabled;
    if (!enabled) {
        s_session = false;
        s_head = s_tail = 0;
        s_num_filter_ids = 0;
    }
    pthread_mutex_unlock(&s_mutex);
}

bool MavlinkCanForward_GetEnabled(void)
{
    pthread_mutex_lock(&s_mutex);
    bool enabled = s_enabled;
    pthread_mutex_unlock(&s_mutex);
    return enabled;
}

void MavlinkCanForward_SetKeepTelemetry(bool keep)
{
    pthread_mutex_lock(&s_mutex);
    s_keep_telemetry = keep;
    pthread_mutex_unlock(&s_mutex);
}

bool MavlinkCanForward_GetKeepTelemetry(void)
{
    pthread_mutex_lock(&s_mutex);
    bool keep = s_keep_telemetry;
    pthread_mutex_unlock(&s_mutex);
    return keep;
}

void MavlinkCanForward_SetForwardBroadcasts(bool forward)
{
    pthread_mutex_lock(&s_mutex);
    s_forward_broadcasts = forward;
    pthread_mutex_unlock(&s_mutex);
}

bool MavlinkCanForward_GetForwardBroadcasts(void)
{
    pthread_mutex_lock(&s_mutex);
    bool forward = s_forward_broadcasts;
    pthread_mutex_unlock(&s_mutex);
    return forward;
}

void MavlinkCanForward_GetStats(can_forward_stats_t *out)
{
    pthread_mutex_lock(&s_mutex);
    *out = s_stats;
    pthread_mutex_unlock(&s_mutex);
}
