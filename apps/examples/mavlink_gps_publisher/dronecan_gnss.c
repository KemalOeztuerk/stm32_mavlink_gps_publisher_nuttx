#include "dronecan_gnss.h"
#include "nav_state.h"
#include "baro_state.h"
#include "ahrs_state.h"
#include "ahrs_filter.h"
#include "clock_ms.h"

#include <canard.h>
#include <canard_nuttx.h>

#include "uavcan.equipment.gnss.Fix2.h"
#include "uavcan.equipment.gnss.Auxiliary.h"
#include "uavcan.equipment.ahrs.MagneticFieldStrength.h"
#include "uavcan.equipment.ahrs.MagneticFieldStrength2.h"
#include "dronecan.sensors.magnetometer.MagneticFieldStrengthHiRes.h"
#include "uavcan.equipment.ahrs.RawIMU.h"
#include "uavcan.equipment.air_data.StaticPressure.h"
#include "uavcan.equipment.air_data.StaticTemperature.h"
#include "uavcan.protocol.NodeStatus.h"
#include "uavcan.protocol.debug.LogMessage.h"
#include "uavcan.protocol.dynamic_node_id.Allocation.h"

#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static CanardInstance s_canard;
static CanardNuttXInstance s_canard_nuttx;
static uint8_t s_canard_memory_pool[CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER_DRONECAN_MEM_POOL_SIZE];

static uint32_t s_boot_ms;
static uint32_t s_last_status_ms;
static uint8_t s_nodestatus_transfer_id;
static uint8_t s_alloc_transfer_id;

/* Node IDs heard on the bus (any non-anonymous transfer), as a 128-bit set.
 * The allocator never hands out an ID it has already heard traffic from. */
static uint32_t s_seen_nodes[4];

/* Diagnostic counters, written only by dronecan_task() and copied out under
 * the mutex for the MAVLink STATUSTEXT diagnostic. */
static dronecan_stats_t s_stats;
static pthread_mutex_t s_stats_mutex;

/* Latest uavcan.protocol.debug.LogMessage from the Here4, forwarded to the
 * GCS as STATUSTEXT by mavlink_tx_task(). Only the most recent message is
 * kept if several arrive between polls. */
static char s_log_text[50];
static bool s_log_pending;

void DroneCanGnss_GetStats(dronecan_stats_t *out)
{
    pthread_mutex_lock(&s_stats_mutex);
    *out = s_stats;
    pthread_mutex_unlock(&s_stats_mutex);
}

bool DroneCanGnss_TakeLogMessage(char *out, size_t out_len)
{
    pthread_mutex_lock(&s_stats_mutex);
    bool pending = s_log_pending;
    if (pending) {
        strlcpy(out, s_log_text, out_len);
        s_log_pending = false;
    }
    pthread_mutex_unlock(&s_stats_mutex);
    return pending;
}

static void stats_lock(void)
{
    pthread_mutex_lock(&s_stats_mutex);
}

static void stats_unlock(void)
{
    pthread_mutex_unlock(&s_stats_mutex);
}

/****************************************************************************
 * Dynamic node ID allocation (DNA) server state
 *
 * Centralized allocator per the DroneCAN specification: an unconfigured
 * allocatee (the Here4) broadcasts its 16-byte unique ID anonymously in up
 * to three 6-byte stages; after each stage we echo the bytes collected so
 * far, and once all 16 have arrived we assign a node ID and broadcast it
 * together with the full unique ID.
 *
 * The allocation table lives in RAM only: after a reboot of this board the
 * Here4 simply re-requests and gets an ID again (deterministically the same
 * one while the table lasts). Only one allocation transaction is tracked at
 * a time -- with several allocatees requesting simultaneously the protocol's
 * random back-off ensures they eventually take turns, per the spec.
 ****************************************************************************/

#define DNA_MAX_NODES    4    /* allocation table size */
#define DNA_MAX_ALLOC_ID 125  /* 126/127 are conventionally left for debug tools */

struct dna_entry {
    bool used;
    uint8_t node_id;
    uint8_t uid[16];
};

static struct dna_entry s_dna_table[DNA_MAX_NODES];
static uint8_t s_dna_uid[16];       /* unique ID collected so far */
static uint8_t s_dna_uid_len;
static uint8_t s_dna_preferred_id;  /* allocatee's preferred ID from stage 1 */
static uint32_t s_dna_last_rx_ms;

/* Drains libcanard's TX queue into the CAN character device. Frames are
 * dropped on hard errors but kept (and retried next cycle) on timeout. */
static void process_tx_queue(void)
{
    const CanardCANFrame *frame;
    while ((frame = canardPeekTxQueue(&s_canard)) != NULL) {
        int res = canardNuttXTransmit(&s_canard_nuttx, frame, 10);
        if (res == 0) {
            break; /* TX path busy: retry the same frame later */
        }
        if (res < 0) {
            stats_lock();
            s_stats.tx_errors++;
            stats_unlock();
        }
        canardPopTxQueue(&s_canard);
    }
}

static float wrap_360(float deg)
{
    deg = fmodf(deg, 360.0f);
    if (deg < 0.0f) {
        deg += 360.0f;
    }
    return deg;
}

/* 1e8-scaled degrees (Fix2's native unit) -> 1e7-scaled degrees (nav_state's
 * unit, matching MAVLink GPS_INPUT/GPS_RAW_INT's lat/lon fields), rounded to
 * the nearest LSB rather than truncated. */
static int32_t scale_deg_1e8_to_1e7(int64_t deg_1e8)
{
    int64_t rounded = (deg_1e8 >= 0) ? (deg_1e8 + 5) / 10 : (deg_1e8 - 5) / 10;
    return (int32_t)rounded;
}

/* Fix2 status -> MAVLink GPS_FIX_TYPE. The scales differ by one at the
 * bottom: Fix2 has no "receiver absent" value (0 already means "receiver
 * present, no lock"), while MAVLink reserves 0 for NO_GPS and uses 1 for
 * NO_FIX. Since decoding a Fix2 at all proves the receiver exists, statuses
 * 0 (no fix) and 1 (time only) both map to MAVLink NO_FIX; a 3D fix is
 * upgraded to DGPS/RTK-float/RTK-fixed from Fix2's mode/sub_mode, matching
 * ArduPilot's AP_GPS_DroneCAN mapping. */
static uint8_t fix2_to_mavlink_fix_type(const struct uavcan_equipment_gnss_Fix2 *msg)
{
    if (msg->status < UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_2D_FIX) {
        return 1; /* GPS_FIX_TYPE_NO_FIX: receiver alive, no lock */
    }
    uint8_t fix_type = msg->status;
    if (msg->status == UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX) {
        if (msg->mode == UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_DGPS) {
            fix_type = 4; /* GPS_FIX_TYPE_DGPS */
        } else if (msg->mode == UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK) {
            fix_type = (msg->sub_mode == UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FIXED)
                           ? 6   /* GPS_FIX_TYPE_RTK_FIXED */
                           : 5;  /* GPS_FIX_TYPE_RTK_FLOAT */
        }
    }
    return fix_type;
}

static void handle_fix2(const CanardRxTransfer *transfer)
{
    struct uavcan_equipment_gnss_Fix2 msg;
    if (uavcan_equipment_gnss_Fix2_decode(transfer, &msg)) {
        return; /* malformed payload */
    }

    bool fix_valid = (msg.status >= UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_2D_FIX);
    int32_t lat_degE7 = scale_deg_1e8_to_1e7(msg.latitude_deg_1e8);
    int32_t lon_degE7 = scale_deg_1e8_to_1e7(msg.longitude_deg_1e8);
    float alt_m = (float)msg.height_msl_mm / 1000.0f;

    float vn = msg.ned_velocity[0];
    float ve = msg.ned_velocity[1];
    float vd = msg.ned_velocity[2];
    float speed_mps = sqrtf(vn * vn + ve * ve);
    float course_deg = wrap_360(atan2f(ve, vn) * (180.0f / (float)M_PI));

    NavState_UpdateGps(fix_valid, fix2_to_mavlink_fix_type(&msg),
                        lat_degE7, lon_degE7,
                        alt_m, speed_mps, course_deg, vd,
                        msg.sats_used, msg.pdop);

    stats_lock();
    s_stats.fix2_count++;
    stats_unlock();
}

static void handle_auxiliary(const CanardRxTransfer *transfer)
{
    struct uavcan_equipment_gnss_Auxiliary msg;
    if (uavcan_equipment_gnss_Auxiliary_decode(transfer, &msg)) {
        return; /* malformed payload */
    }

    NavState_UpdateDops(msg.hdop, msg.vdop, msg.sats_used, msg.sats_visible);
}

/* Tilt-compensated heading from raw body-frame magnetometer readings plus
 * the current roll/pitch estimate. No hard-iron/soft-iron calibration is
 * applied (offsets are implicitly zero). Assumes X-forward/Y-right/Z-down
 * body frame, matching the DroneCAN convention.
 *
 * If the Here4 isn't broadcasting RawIMU (a firmware option that is often
 * off), there is no attitude estimate to compensate with; the unit is then
 * assumed level (roll = pitch = 0), which is exact for a stationary
 * horizontal mount and a reasonable approximation otherwise. */
static void update_heading_from_field(float mx, float my, float mz)
{
    ahrs_state_t ahrs;
    AhrsState_GetSnapshot(&ahrs);
    float roll_rad = ahrs.valid ? ahrs.roll_rad : 0.0f;
    float pitch_rad = ahrs.valid ? ahrs.pitch_rad : 0.0f;

    float cr = cosf(roll_rad);
    float sr = sinf(roll_rad);
    float cp = cosf(pitch_rad);
    float sp = sinf(pitch_rad);

    float xh = mx * cp + my * sr * sp + mz * cr * sp;
    float yh = my * cr - mz * sr;

    float heading_deg = wrap_360(atan2f(yh, xh) * (180.0f / (float)M_PI));
    NavState_UpdateHeading(heading_deg);
    AhrsFilter_SetMagHeading(heading_deg);

    stats_lock();
    s_stats.mag_count++;
    stats_unlock();
}

/* AP_Periph peripherals (the Here4 included) broadcast the original
 * MagneticFieldStrength unless built with the magnetic-survey (HiRes)
 * option; MagneticFieldStrength2 is kept for peripherals that send the
 * per-sensor-ID variant. All three feed the same heading computation. */
static void handle_mag_old(const CanardRxTransfer *transfer)
{
    struct uavcan_equipment_ahrs_MagneticFieldStrength msg;
    if (uavcan_equipment_ahrs_MagneticFieldStrength_decode(transfer, &msg)) {
        return; /* malformed payload */
    }
    update_heading_from_field(msg.magnetic_field_ga[0], msg.magnetic_field_ga[1],
                               msg.magnetic_field_ga[2]);
}

static void handle_mag(const CanardRxTransfer *transfer)
{
    struct uavcan_equipment_ahrs_MagneticFieldStrength2 msg;
    if (uavcan_equipment_ahrs_MagneticFieldStrength2_decode(transfer, &msg)) {
        return; /* malformed payload */
    }
    update_heading_from_field(msg.magnetic_field_ga[0], msg.magnetic_field_ga[1],
                               msg.magnetic_field_ga[2]);
}

static void handle_mag_hires(const CanardRxTransfer *transfer)
{
    struct dronecan_sensors_magnetometer_MagneticFieldStrengthHiRes msg;
    if (dronecan_sensors_magnetometer_MagneticFieldStrengthHiRes_decode(transfer, &msg)) {
        return; /* malformed payload */
    }
    update_heading_from_field(msg.magnetic_field_ga[0], msg.magnetic_field_ga[1],
                               msg.magnetic_field_ga[2]);
}

static void handle_raw_imu(const CanardRxTransfer *transfer)
{
    struct uavcan_equipment_ahrs_RawIMU msg;
    if (uavcan_equipment_ahrs_RawIMU_decode(transfer, &msg)) {
        return; /* malformed payload */
    }

    AhrsFilter_Update(msg.accelerometer_latest[0], msg.accelerometer_latest[1],
                       msg.accelerometer_latest[2],
                       msg.rate_gyro_latest[0], msg.rate_gyro_latest[1],
                       msg.rate_gyro_latest[2]);

    stats_lock();
    for (int i = 0; i < 3; i++) {
        s_stats.raw_acc[i] = (int16_t)(msg.accelerometer_latest[i] * 10.0f);
    }
    stats_unlock();
}

static void handle_static_pressure(const CanardRxTransfer *transfer)
{
    struct uavcan_equipment_air_data_StaticPressure msg;
    if (uavcan_equipment_air_data_StaticPressure_decode(transfer, &msg)) {
        return; /* malformed payload */
    }

    BaroState_UpdatePressure(msg.static_pressure);
}

static void handle_static_temperature(const CanardRxTransfer *transfer)
{
    struct uavcan_equipment_air_data_StaticTemperature msg;
    if (uavcan_equipment_air_data_StaticTemperature_decode(transfer, &msg)) {
        return; /* malformed payload */
    }

    BaroState_UpdateTemperature(msg.static_temperature - 273.15f); /* K -> degC */
}

/****************************************************************************
 * DNA server
 ****************************************************************************/

static bool dna_id_taken(uint8_t id)
{
    if (id == canardGetLocalNodeID(&s_canard)) {
        return true;
    }
    if (s_seen_nodes[id / 32] & (1U << (id % 32))) {
        return true;
    }
    for (int i = 0; i < DNA_MAX_NODES; i++) {
        if (s_dna_table[i].used && s_dna_table[i].node_id == id) {
            return true;
        }
    }
    return false;
}

/* Returns the node ID to assign for this unique ID, or 0 if none available.
 * An ID already allocated to the same unique ID is returned unchanged, so a
 * rebooting Here4 keeps its ID for as long as this board stays up. */
static uint8_t dna_pick_node_id(const uint8_t uid[16], uint8_t preferred)
{
    for (int i = 0; i < DNA_MAX_NODES; i++) {
        if (s_dna_table[i].used && memcmp(s_dna_table[i].uid, uid, 16) == 0) {
            return s_dna_table[i].node_id;
        }
    }

    uint8_t candidate = 0;
    if (preferred >= 1 && preferred <= DNA_MAX_ALLOC_ID && !dna_id_taken(preferred)) {
        candidate = preferred;
    } else {
        for (uint8_t id = DNA_MAX_ALLOC_ID; id >= 1; id--) {
            if (!dna_id_taken(id)) {
                candidate = id;
                break;
            }
        }
    }
    if (candidate == 0) {
        return 0; /* bus full */
    }

    for (int i = 0; i < DNA_MAX_NODES; i++) {
        if (!s_dna_table[i].used) {
            s_dna_table[i].used = true;
            s_dna_table[i].node_id = candidate;
            memcpy(s_dna_table[i].uid, uid, 16);
            return candidate;
        }
    }
    return 0; /* table full */
}

/* Broadcasts an Allocation message: during the handshake node_id is 0 and
 * unique_id echoes the bytes collected so far; on completion node_id is the
 * assigned ID and unique_id is the allocatee's full 16 bytes. */
static void dna_send_response(uint8_t node_id, const uint8_t *uid, uint8_t uid_len)
{
    struct uavcan_protocol_dynamic_node_id_Allocation rsp;
    uint8_t buf[UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_SIZE];

    memset(&rsp, 0, sizeof(rsp));
    rsp.node_id = node_id;
    rsp.first_part_of_unique_id = false;
    rsp.unique_id.len = uid_len;
    memcpy(rsp.unique_id.data, uid, uid_len);

    uint32_t len = uavcan_protocol_dynamic_node_id_Allocation_encode(&rsp, buf);
    canardBroadcast(&s_canard,
                    UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
                    UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
                    &s_alloc_transfer_id, CANARD_TRANSFER_PRIORITY_HIGH,
                    buf, (uint16_t)len);
    process_tx_queue();
}

static void handle_allocation(const CanardRxTransfer *transfer)
{
    if (transfer->source_node_id != CANARD_BROADCAST_NODE_ID) {
        return; /* another allocator's response, not an allocatee request */
    }

    struct uavcan_protocol_dynamic_node_id_Allocation msg;
    if (uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &msg)) {
        return; /* malformed payload */
    }
    if (msg.unique_id.len == 0 ||
        msg.unique_id.len > UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_LENGTH_OF_UNIQUE_ID_IN_REQUEST) {
        return; /* allocatee stages carry 1..6 unique ID bytes */
    }

    stats_lock();
    s_stats.alloc_requests++;
    stats_unlock();

    uint32_t now = clock_now_ms();
    if (msg.first_part_of_unique_id) {
        s_dna_uid_len = 0;
        s_dna_preferred_id = msg.node_id;
    } else if (s_dna_uid_len == 0 ||
               (now - s_dna_last_rx_ms) > UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_FOLLOWUP_TIMEOUT_MS) {
        s_dna_uid_len = 0; /* stray/expired follow-up: wait for a fresh first stage */
        return;
    }

    if ((size_t)s_dna_uid_len + msg.unique_id.len > sizeof(s_dna_uid)) {
        s_dna_uid_len = 0; /* protocol violation */
        return;
    }
    memcpy(&s_dna_uid[s_dna_uid_len], msg.unique_id.data, msg.unique_id.len);
    s_dna_uid_len += msg.unique_id.len;
    s_dna_last_rx_ms = now;

    if (s_dna_uid_len < sizeof(s_dna_uid)) {
        /* Echo what we have so far; the allocatee answers with the next
         * stage once it sees its own prefix. */
        dna_send_response(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ANY_NODE_ID,
                          s_dna_uid, s_dna_uid_len);
        return;
    }

    uint8_t assigned = dna_pick_node_id(s_dna_uid, s_dna_preferred_id);
    s_dna_uid_len = 0;
    if (assigned != 0) {
        dna_send_response(assigned, s_dna_uid, sizeof(s_dna_uid));
        stats_lock();
        s_stats.allocated_id = assigned;
        stats_unlock();
    }
}

static void handle_node_status(const CanardRxTransfer *transfer)
{
    struct uavcan_protocol_NodeStatus msg;
    if (uavcan_protocol_NodeStatus_decode(transfer, &msg)) {
        return; /* malformed payload */
    }

    /* Surface the peripheral's self-reported health (0=OK 1=warn 2=error
     * 3=critical) in the STATUSTEXT diagnostic. */
    stats_lock();
    s_stats.remote_health = msg.health;
    stats_unlock();
}

/* The Here4 (AP_Periph) broadcasts its internal printf/error output as
 * LogMessage; forwarding it to the GCS gives direct visibility into e.g.
 * GPS probe failures without any debug connection to the Here4. */
static void handle_log_message(const CanardRxTransfer *transfer)
{
    struct uavcan_protocol_debug_LogMessage msg;
    if (uavcan_protocol_debug_LogMessage_decode(transfer, &msg)) {
        return; /* malformed payload */
    }

    /* "H4[" + src(9) + "]" + text(36) fits the 50-byte STATUSTEXT exactly. */
    char src[10];
    size_t src_len = msg.source.len < sizeof(src) - 1 ? msg.source.len : sizeof(src) - 1;
    memcpy(src, msg.source.data, src_len);
    src[src_len] = '\0';

    char text[37];
    size_t text_len = msg.text.len < sizeof(text) - 1 ? msg.text.len : sizeof(text) - 1;
    memcpy(text, msg.text.data, text_len);
    text[text_len] = '\0';

    stats_lock();
    snprintf(s_log_text, sizeof(s_log_text), "H4[%s]%s", src, text);
    s_log_pending = true;
    stats_unlock();
}

static void broadcast_node_status(void)
{
    struct uavcan_protocol_NodeStatus status;
    uint8_t buf[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];

    memset(&status, 0, sizeof(status));
    status.uptime_sec = (clock_now_ms() - s_boot_ms) / 1000U;
    status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;

    uint32_t len = uavcan_protocol_NodeStatus_encode(&status, buf);
    canardBroadcast(&s_canard,
                    UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    &s_nodestatus_transfer_id, CANARD_TRANSFER_PRIORITY_LOW,
                    buf, (uint16_t)len);
    process_tx_queue();
}

static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
    (void)ins;

    if (transfer->source_node_id != CANARD_BROADCAST_NODE_ID) {
        s_seen_nodes[transfer->source_node_id / 32] |=
            1U << (transfer->source_node_id % 32);
        stats_lock();
        s_stats.remote_node_id = transfer->source_node_id;
        stats_unlock();
    }

    switch (transfer->data_type_id) {
    case UAVCAN_EQUIPMENT_GNSS_FIX2_ID:
        handle_fix2(transfer);
        break;
    case UAVCAN_EQUIPMENT_GNSS_AUXILIARY_ID:
        handle_auxiliary(transfer);
        break;
    case UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_ID:
        handle_mag_old(transfer);
        break;
    case UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_ID:
        handle_mag(transfer);
        break;
    case DRONECAN_SENSORS_MAGNETOMETER_MAGNETICFIELDSTRENGTHHIRES_ID:
        handle_mag_hires(transfer);
        break;
    case UAVCAN_EQUIPMENT_AHRS_RAWIMU_ID:
        handle_raw_imu(transfer);
        break;
    case UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_ID:
        handle_static_pressure(transfer);
        break;
    case UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE_ID:
        handle_static_temperature(transfer);
        break;
    case UAVCAN_PROTOCOL_NODESTATUS_ID:
        handle_node_status(transfer);
        break;
    case UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_ID:
        handle_log_message(transfer);
        break;
    case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID:
        handle_allocation(transfer);
        break;
    default:
        break;
    }
}

static bool shouldAcceptTransfer(const CanardInstance *ins,
                                  uint64_t *out_data_type_signature,
                                  uint16_t data_type_id,
                                  CanardTransferType transfer_type,
                                  uint8_t source_node_id)
{
    (void)ins;
    (void)source_node_id;

    if (transfer_type != CanardTransferTypeBroadcast) {
        return false;
    }

    switch (data_type_id) {
    case UAVCAN_EQUIPMENT_GNSS_FIX2_ID:
        *out_data_type_signature = UAVCAN_EQUIPMENT_GNSS_FIX2_SIGNATURE;
        return true;
    case UAVCAN_EQUIPMENT_GNSS_AUXILIARY_ID:
        *out_data_type_signature = UAVCAN_EQUIPMENT_GNSS_AUXILIARY_SIGNATURE;
        return true;
    case UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_ID:
        *out_data_type_signature = UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_SIGNATURE;
        return true;
    case UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_ID:
        *out_data_type_signature = UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_SIGNATURE;
        return true;
    case DRONECAN_SENSORS_MAGNETOMETER_MAGNETICFIELDSTRENGTHHIRES_ID:
        *out_data_type_signature = DRONECAN_SENSORS_MAGNETOMETER_MAGNETICFIELDSTRENGTHHIRES_SIGNATURE;
        return true;
    case UAVCAN_EQUIPMENT_AHRS_RAWIMU_ID:
        *out_data_type_signature = UAVCAN_EQUIPMENT_AHRS_RAWIMU_SIGNATURE;
        return true;
    case UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_ID:
        *out_data_type_signature = UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_SIGNATURE;
        return true;
    case UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE_ID:
        *out_data_type_signature = UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE_SIGNATURE;
        return true;
    case UAVCAN_PROTOCOL_NODESTATUS_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE;
        return true;
    case UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_SIGNATURE;
        return true;
    /* Anonymous allocation requests arrive with the data type ID truncated
     * to its low 2 bits (== 1, Allocation's full ID) per the DroneCAN
     * anonymous frame format, so this case matches both. */
    case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
        return true;
    default:
        return false;
    }
}

int DroneCanGnss_Init(const char *devpath)
{
    if (canardNuttXInit(&s_canard_nuttx, devpath) < 0) {
        return -1;
    }

    pthread_mutex_init(&s_stats_mutex, NULL);

    canardInit(&s_canard, s_canard_memory_pool, sizeof(s_canard_memory_pool),
               onTransferReceived, shouldAcceptTransfer, NULL);
    canardSetLocalNodeID(&s_canard, CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER_DRONECAN_NODE_ID);

    s_boot_ms = clock_now_ms();
    s_last_status_ms = s_boot_ms;

    return 0;
}

void *dronecan_task(void *argument)
{
    (void)argument;
    for (;;) {
        CanardCANFrame rx_frame;
        int rx_res = canardNuttXReceive(&s_canard_nuttx, &rx_frame, 100);
        uint64_t now_us = (uint64_t)clock_now_ms() * 1000ULL;
        if (rx_res > 0) {
            stats_lock();
            s_stats.rx_frames++;
            stats_unlock();
            canardHandleRxFrame(&s_canard, &rx_frame, now_us);
        }

        uint32_t now = clock_now_ms();

        /* Abandon a half-collected unique ID if the allocatee went quiet. */
        if (s_dna_uid_len > 0 &&
            (now - s_dna_last_rx_ms) > UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_FOLLOWUP_TIMEOUT_MS) {
            s_dna_uid_len = 0;
        }

        if ((now - s_last_status_ms) >= 1000U) {
            s_last_status_ms = now;
            broadcast_node_status();
            canardCleanupStaleTransfers(&s_canard, now_us);
        }

        process_tx_queue();
    }

    return NULL;
}
