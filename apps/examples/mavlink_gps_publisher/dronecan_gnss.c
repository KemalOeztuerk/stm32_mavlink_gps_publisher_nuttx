#include "dronecan_gnss.h"
#include "nav_state.h"
#include "imu_state.h"
#include "ahrs_state.h"
#include "ahrs_filter.h"
#include "clock_ms.h"

#include <canard.h>
#include <canard_nuttx.h>

#include "uavcan.equipment.gnss.Fix2.h"
#include "uavcan.equipment.ahrs.MagneticFieldStrength2.h"
#include "uavcan.equipment.ahrs.RawIMU.h"

#include <math.h>
#include <stdint.h>

static CanardInstance s_canard;
static CanardNuttXInstance s_canard_nuttx;
static uint8_t s_canard_memory_pool[CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER_DRONECAN_MEM_POOL_SIZE];

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
    float speed_mps = sqrtf(vn * vn + ve * ve);
    float course_deg = wrap_360(atan2f(ve, vn) * (180.0f / (float)M_PI));

    /* Fix2 has no direct HDOP field; pdop is the closest available
     * precision figure and is used here as an approximation. */
    NavState_UpdateGps(fix_valid, msg.status, lat_degE7, lon_degE7,
                        alt_m, speed_mps, course_deg,
                        msg.sats_used, msg.pdop, NAV_GPS_SOURCE_CAN);
}

/* Tilt-compensated heading from raw body-frame magnetometer readings plus
 * the current roll/pitch estimate. No hard-iron/soft-iron calibration is
 * applied (offsets are implicitly zero) -- this board never had a
 * magnetometer before the Here4, so an uncalibrated first pass is new
 * capability, not a regression. Assumes X-forward/Y-right/Z-down body frame,
 * matching the convention already documented in mpu9250.c. */
static void handle_mag(const CanardRxTransfer *transfer)
{
    struct uavcan_equipment_ahrs_MagneticFieldStrength2 msg;
    if (uavcan_equipment_ahrs_MagneticFieldStrength2_decode(transfer, &msg)) {
        return; /* malformed payload */
    }

    ahrs_state_t ahrs;
    AhrsState_GetSnapshot(&ahrs);
    if (!ahrs.valid) {
        return; /* no attitude yet: can't tilt-compensate */
    }

    float mx = msg.magnetic_field_ga[0];
    float my = msg.magnetic_field_ga[1];
    float mz = msg.magnetic_field_ga[2];

    float cr = cosf(ahrs.roll_rad);
    float sr = sinf(ahrs.roll_rad);
    float cp = cosf(ahrs.pitch_rad);
    float sp = sinf(ahrs.pitch_rad);

    float xh = mx * cp + my * sr * sp + mz * cr * sp;
    float yh = my * cr - mz * sr;

    float heading_deg = wrap_360(atan2f(yh, xh) * (180.0f / (float)M_PI));
    NavState_UpdateHeading(heading_deg);
}

/* Feeds the Here4's raw accel/gyro into the shared AHRS filter, taking
 * priority over the onboard MPU9250 while it keeps updating -- see
 * ahrs_filter.c. RawIMU has no temperature field, so the last known
 * MPU9250 reading is passed through unchanged rather than a fabricated
 * value. */
static void handle_raw_imu(const CanardRxTransfer *transfer)
{
    struct uavcan_equipment_ahrs_RawIMU msg;
    if (uavcan_equipment_ahrs_RawIMU_decode(transfer, &msg)) {
        return; /* malformed payload */
    }

    imu_state_t cur;
    ImuState_GetSnapshot(&cur);

    AhrsFilter_Update(msg.accelerometer_latest[0], msg.accelerometer_latest[1],
                       msg.accelerometer_latest[2],
                       msg.rate_gyro_latest[0], msg.rate_gyro_latest[1],
                       msg.rate_gyro_latest[2],
                       cur.temperature_degc, AHRS_SOURCE_CAN);
}

static void onTransferReceived(CanardInstance *ins, CanardRxTransfer *transfer)
{
    (void)ins;
    switch (transfer->data_type_id) {
    case UAVCAN_EQUIPMENT_GNSS_FIX2_ID:
        handle_fix2(transfer);
        break;
    case UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_ID:
        handle_mag(transfer);
        break;
    case UAVCAN_EQUIPMENT_AHRS_RAWIMU_ID:
        handle_raw_imu(transfer);
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
    case UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_ID:
        *out_data_type_signature = UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH2_SIGNATURE;
        return true;
    case UAVCAN_EQUIPMENT_AHRS_RAWIMU_ID:
        *out_data_type_signature = UAVCAN_EQUIPMENT_AHRS_RAWIMU_SIGNATURE;
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

    canardInit(&s_canard, s_canard_memory_pool, sizeof(s_canard_memory_pool),
               onTransferReceived, shouldAcceptTransfer, NULL);
    canardSetLocalNodeID(&s_canard, CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER_DRONECAN_NODE_ID);

    return 0;
}

void *dronecan_task(void *argument)
{
    (void)argument;
    for (;;) {
        CanardCANFrame rx_frame;
        int rx_res = canardNuttXReceive(&s_canard_nuttx, &rx_frame, 1000);
        if (rx_res > 0) {
            canardHandleRxFrame(&s_canard, &rx_frame, (uint64_t)clock_now_ms() * 1000ULL);
        }
    }

    return NULL;
}
