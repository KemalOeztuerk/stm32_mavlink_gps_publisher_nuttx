#include "mavlink_link.h"
#include "nav_state.h"
#include "imu_state.h"
#include "ahrs_state.h"
#include "baro_state.h"
#include "dronecan_gnss.h"
#include "mavlink_can_forward.h"
#include "clock_ms.h"
#include "common/mavlink.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static int s_fd = -1;
static pthread_mutex_t s_tx_mutex;

int MavlinkLink_Init(const char *devpath)
{
    pthread_mutex_init(&s_tx_mutex, NULL);

    int fd = open(devpath, O_RDWR);
    if (fd < 0) {
        return -1;
    }

    struct termios tio;
    if (tcgetattr(fd, &tio) == 0) {
        cfsetspeed(&tio, CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER_MAVLINK_BAUD);
        tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
        tio.c_cflag |= CS8;
        tcsetattr(fd, TCSANOW, &tio);
    }

    s_fd = fd;
    return 0;
}

/* Serialises the UART. mavlink_tx_task() (telemetry plus forwarded CAN
 * frames) and mavlink_rx_task() (command acks, parameter replies) both send,
 * and two half-interleaved MAVLink frames are two unparseable ones. */
void MavlinkLink_Send(const mavlink_message_t *msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);

    pthread_mutex_lock(&s_tx_mutex);
    write(s_fd, buf, len);
    pthread_mutex_unlock(&s_tx_mutex);
}

static void send_message(const mavlink_message_t *msg)
{
    MavlinkLink_Send(msg);
}

/* Forwarded CAN frames are packed here, not in mavlink_can_forward.c, and
 * the reason is subtle: MAVLink's per-channel TX sequence counter lives in a
 * function-local static inside a static-inline helper, so every translation
 * unit that packs a message gets its own copy of it. Two counters
 * interleaved on one link look like heavy packet loss to a GCS. Keeping all
 * packing in this file keeps the sequence numbers monotonic. */
void MavlinkLink_SendCanFrame(uint8_t target_system, uint8_t target_component,
                               uint8_t bus, uint8_t len, uint32_t id,
                               const uint8_t *data)
{
    mavlink_message_t msg;
    mavlink_msg_can_frame_pack(MAVLINK_LINK_SYSTEM_ID, MAVLINK_LINK_COMPONENT_ID, &msg,
                                target_system, target_component, bus, len, id,
                                data);
    send_message(&msg);
}

static uint16_t heading_to_mavlink_yaw(bool valid, float heading_deg)
{
    if (!valid) {
        return 0U; /* not available */
    }
    uint16_t cdeg = (uint16_t)(heading_deg * 100.0f);
    if (cdeg == 0U) {
        cdeg = 36000U; /* 0 is reserved to mean "unavailable"; true north is 36000 */
    }
    return cdeg;
}

static void send_heartbeat(void)
{
    mavlink_message_t msg;
    /* Declared as a ground rover so a GCS draws it on the map as a vehicle.
     * The autopilot field stays MAV_AUTOPILOT_INVALID, which is the honest
     * answer -- there is no autopilot here -- at the cost of the GCS showing
     * the flight mode as "Unknown". */
    mavlink_msg_heartbeat_pack(MAVLINK_LINK_SYSTEM_ID, MAVLINK_LINK_COMPONENT_ID, &msg,
                                MAV_TYPE_GROUND_ROVER, MAV_AUTOPILOT_INVALID,
                                0, 0, MAV_STATE_ACTIVE);
    send_message(&msg);
}

static void send_gps_input(const nav_state_t *nav)
{
    float course_rad = nav->course_deg * ((float)M_PI / 180.0f);
    float vn = nav->speed_mps * cosf(course_rad);
    float ve = nav->speed_mps * sinf(course_rad);

    /* Fix2 always provides the full NED velocity; HDOP/VDOP come from the
     * separate gnss.Auxiliary broadcast, with Fix2's PDOP standing in for
     * HDOP until Auxiliary has been seen. */
    uint16_t ignore_flags = GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                             GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                             GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY;
    float hdop = nav->dop_valid ? nav->hdop : nav->pdop;
    float vdop = nav->vdop;
    if (!nav->dop_valid) {
        ignore_flags |= GPS_INPUT_IGNORE_FLAG_VDOP;
    }

    mavlink_message_t msg;
    mavlink_msg_gps_input_pack(MAVLINK_LINK_SYSTEM_ID, MAVLINK_LINK_COMPONENT_ID, &msg,
                                (uint64_t)clock_now_ms() * 1000ULL, /* time_usec, since-boot */
                                0,                                 /* gps_id */
                                ignore_flags,
                                0, 0,                               /* time_week_ms, time_week: not tracked */
                                nav->fix_type,
                                nav->lat_degE7, nav->lon_degE7,
                                nav->alt_m,
                                hdop, vdop,
                                vn, ve, nav->vel_d_mps,
                                0.0f, 0.0f, 0.0f,                   /* speed/h/v accuracy (ignored) */
                                nav->sats_used,
                                heading_to_mavlink_yaw(nav->heading_valid, nav->heading_deg));
    send_message(&msg);
}

/* Debug-only: GCS software (Mission Planner, QGC) plots vehicle position from
 * GPS_RAW_INT/GLOBAL_POSITION_INT, not from GPS_INPUT (which is an injection
 * message for an autopilot's internal EKF, not a display message). Sending
 * this too lets you see the fix on the map during bench testing without a
 * real flight controller in the loop; the real integration only needs
 * GPS_INPUT. */
static void send_gps_raw_int(const nav_state_t *nav)
{
    float hdop = nav->dop_valid ? nav->hdop : nav->pdop;
    uint16_t eph = (hdop > 0.0f) ? (uint16_t)(hdop * 100.0f) : UINT16_MAX;
    uint16_t epv = (nav->dop_valid && nav->vdop > 0.0f)
                       ? (uint16_t)(nav->vdop * 100.0f) : UINT16_MAX;
    /* Prefer the receiver's visible count, but some Here4 firmwares report
     * 0 visible in Auxiliary even while Fix2 shows satellites in use --
     * fall back so a 3D fix never displays alongside "0 satellites". */
    uint8_t sats = (nav->dop_valid && nav->sats_visible > 0)
                       ? nav->sats_visible : nav->sats_used;

    mavlink_message_t msg;
    mavlink_msg_gps_raw_int_pack(MAVLINK_LINK_SYSTEM_ID, MAVLINK_LINK_COMPONENT_ID, &msg,
                                  (uint64_t)clock_now_ms() * 1000ULL,
                                  nav->fix_type,
                                  nav->lat_degE7, nav->lon_degE7,
                                  (int32_t)(nav->alt_m * 1000.0f),
                                  eph, epv,
                                  (uint16_t)(nav->speed_mps * 100.0f),
                                  (uint16_t)(nav->course_deg * 100.0f),
                                  sats,
                                  0, 0, 0, 0, 0, /* alt_ellipsoid/h_acc/v_acc/vel_acc/hdg_acc: not available */
                                  heading_to_mavlink_yaw(nav->heading_valid, nav->heading_deg));
    send_message(&msg);
}

/* Mag fields are left at 0 and unflagged: the Here4's magnetometer feeds
 * the tilt-compensated heading in nav_state instead of being re-broadcast
 * raw here. Pressure/temperature come from the Here4's barometer when it
 * broadcasts one (StaticPressure/StaticTemperature). */
static void send_highres_imu(const imu_state_t *imu, const baro_state_t *baro)
{
    if (!imu->valid) {
        return;
    }

    uint16_t fields_updated = HIGHRES_IMU_UPDATED_XACC | HIGHRES_IMU_UPDATED_YACC |
                               HIGHRES_IMU_UPDATED_ZACC | HIGHRES_IMU_UPDATED_XGYRO |
                               HIGHRES_IMU_UPDATED_YGYRO | HIGHRES_IMU_UPDATED_ZGYRO;

    float abs_pressure_hpa = 0.0f;
    float pressure_alt_m = 0.0f;
    if (baro->pressure_valid) {
        abs_pressure_hpa = baro->pressure_pa / 100.0f;
        /* ISA barometric altitude from absolute pressure. */
        pressure_alt_m = 44330.0f * (1.0f - powf(baro->pressure_pa / 101325.0f, 0.190295f));
        fields_updated |= HIGHRES_IMU_UPDATED_ABS_PRESSURE |
                          HIGHRES_IMU_UPDATED_PRESSURE_ALT;
    }

    float temperature_degc = 0.0f;
    if (baro->temperature_valid) {
        temperature_degc = baro->temperature_degc;
        fields_updated |= HIGHRES_IMU_UPDATED_TEMPERATURE;
    }

    mavlink_message_t msg;
    mavlink_msg_highres_imu_pack(MAVLINK_LINK_SYSTEM_ID, MAVLINK_LINK_COMPONENT_ID, &msg,
                                  (uint64_t)clock_now_ms() * 1000ULL,
                                  imu->accel_x_mps2, imu->accel_y_mps2, imu->accel_z_mps2,
                                  imu->gyro_x_rads, imu->gyro_y_rads, imu->gyro_z_rads,
                                  0.0f, 0.0f, 0.0f,             /* xmag/ymag/zmag: not re-broadcast */
                                  abs_pressure_hpa, 0.0f, pressure_alt_m,
                                  temperature_degc,
                                  fields_updated, 0);
    send_message(&msg);
}

/* Here4 barometer passthrough for GCS display / logging. */
static void send_scaled_pressure(const baro_state_t *baro)
{
    if (!baro->pressure_valid && !baro->temperature_valid) {
        return;
    }

    mavlink_message_t msg;
    mavlink_msg_scaled_pressure_pack(MAVLINK_LINK_SYSTEM_ID, MAVLINK_LINK_COMPONENT_ID, &msg,
                                      clock_now_ms(),
                                      baro->pressure_valid ? baro->pressure_pa / 100.0f : 0.0f,
                                      0.0f, /* differential pressure: no pitot */
                                      baro->temperature_valid
                                          ? (int16_t)(baro->temperature_degc * 100.0f) : 0,
                                      0);   /* diff-pressure temperature: not available */
    send_message(&msg);
}

/* FOLLOW_TARGET capability bits, matching ArduPilot's
 * libraries/AP_Scripting/applets/follow-target-send.lua FOLLOW_TARGET_CAPABILITIES table. */
#define FOLLOW_TARGET_CAP_POS        (1U << 0)
#define FOLLOW_TARGET_CAP_VEL        (1U << 1)
#define FOLLOW_TARGET_CAP_ACCEL      (1U << 2)
#define FOLLOW_TARGET_CAP_ATT_RATES  (1U << 3)

/* Standard aerospace ZYX Euler -> quaternion (q1=w, q2=x, q3=y, q4=z),
 * matching ArduPilot's Quaternion::from_euler(). */
static void euler_to_quaternion(float roll, float pitch, float yaw, float q[4])
{
    float cr = cosf(roll * 0.5f);
    float sr = sinf(roll * 0.5f);
    float cp = cosf(pitch * 0.5f);
    float sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw * 0.5f);
    float sy = sinf(yaw * 0.5f);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

/* Rotates a body-frame vector into the earth (NED) frame using the current
 * roll/pitch/yaw estimate, matching ArduPilot's AP_AHRS::body_to_earth() used
 * by the reference script to convert gyro rates before sending them. */
static void body_to_earth(float roll, float pitch, float yaw,
                           float bx, float by, float bz, float earth[3])
{
    float cr = cosf(roll);
    float sr = sinf(roll);
    float cp = cosf(pitch);
    float sp = sinf(pitch);
    float cy = cosf(yaw);
    float sy = sinf(yaw);

    earth[0] = (cy * cp) * bx + (cy * sp * sr - sy * cr) * by + (cy * sp * cr + sy * sr) * bz;
    earth[1] = (sy * cp) * bx + (sy * sp * sr + cy * cr) * by + (sy * sp * cr - cy * sr) * bz;
    earth[2] = (-sp) * bx + (cp * sr) * by + (cp * cr) * bz;
}

/* Lets an ArduPilot vehicle in FOLLOW mode track this board, mirroring
 * libraries/AP_Scripting/applets/follow-target-send.lua's FOLLOW_TARGET
 * message. Skipped entirely without a GPS fix, matching that script's
 * early-return when ahrs:get_location() is nil. No position controller here,
 * so ACCEL capability/field is always left unset/zero (this board has no
 * target-acceleration source, unlike the vehicle poscontrol the script reads
 * from); VEL uses our actual GPS ground velocity as the closest equivalent. */
static void send_follow_target(const nav_state_t *nav, const ahrs_state_t *ahrs)
{
    if (!nav->gps_fix_valid) {
        return;
    }

    uint8_t capabilities = FOLLOW_TARGET_CAP_POS | FOLLOW_TARGET_CAP_VEL;

    float course_rad = nav->course_deg * ((float)M_PI / 180.0f);
    float vel[3] = { nav->speed_mps * cosf(course_rad), nav->speed_mps * sinf(course_rad),
                     nav->vel_d_mps };
    float acc[3] = { 0.0f, 0.0f, 0.0f };

    float attitude_q[4] = { 1.0f, 0.0f, 0.0f, 0.0f }; /* identity: unknown */
    float rates[3] = { 0.0f, 0.0f, 0.0f };
    if (ahrs->valid) {
        capabilities |= FOLLOW_TARGET_CAP_ATT_RATES;
        euler_to_quaternion(ahrs->roll_rad, ahrs->pitch_rad, ahrs->yaw_rad, attitude_q);
        body_to_earth(ahrs->roll_rad, ahrs->pitch_rad, ahrs->yaw_rad,
                       ahrs->rollspeed_rads, ahrs->pitchspeed_rads, ahrs->yawspeed_rads, rates);
    }

    float position_cov[3] = { 0.0f, 0.0f, 0.0f }; /* unknown, matches upstream script */

    mavlink_message_t msg;
    mavlink_msg_follow_target_pack(MAVLINK_LINK_SYSTEM_ID, MAVLINK_LINK_COMPONENT_ID, &msg,
                                    (uint64_t)clock_now_ms(), capabilities,
                                    nav->lat_degE7, nav->lon_degE7, nav->alt_m,
                                    vel, acc, attitude_q, rates, position_cov,
                                    0ULL);
    send_message(&msg);
}

static int16_t rad_to_heading_deg(float yaw_rad)
{
    float deg = yaw_rad * (180.0f / (float)M_PI);
    deg = fmodf(deg, 360.0f);
    if (deg < 0.0f) {
        deg += 360.0f;
    }
    return (int16_t)deg;
}

/* Drives the GCS artificial horizon. */
static void send_attitude(const ahrs_state_t *ahrs)
{
    if (!ahrs->valid) {
        return;
    }
    mavlink_message_t msg;
    mavlink_msg_attitude_pack(MAVLINK_LINK_SYSTEM_ID, MAVLINK_LINK_COMPONENT_ID, &msg,
                               clock_now_ms(),
                               ahrs->roll_rad, ahrs->pitch_rad, ahrs->yaw_rad,
                               ahrs->rollspeed_rads, ahrs->pitchspeed_rads, ahrs->yawspeed_rads);
    send_message(&msg);
}

/* Drives the GCS heading tape / speed / altitude tapes. airspeed/throttle
 * are 0: no pitot, no ESC telemetry. Climb rate is the negated Fix2 NED
 * down velocity.
 *
 * Heading prefers the AHRS yaw, but falls back to the compass heading when
 * there is no attitude estimate. That case is not exotic: a Here4 with
 * IMU_SAMPLE_RATE=0 never broadcasts RawIMU, so the AHRS never initialises
 * and this would otherwise report a hard 0 while a perfectly good
 * tilt-compensated heading sat in nav_state going nowhere. */
static void send_vfr_hud(const nav_state_t *nav, const ahrs_state_t *ahrs)
{
    int16_t heading_deg = 0;
    if (ahrs->valid) {
        heading_deg = rad_to_heading_deg(ahrs->yaw_rad);
    } else if (nav->heading_valid) {
        heading_deg = (int16_t)nav->heading_deg;
    }

    mavlink_message_t msg;
    mavlink_msg_vfr_hud_pack(MAVLINK_LINK_SYSTEM_ID, MAVLINK_LINK_COMPONENT_ID, &msg,
                              0.0f, nav->speed_mps, heading_deg, 0, nav->alt_m,
                              -nav->vel_d_mps);
    send_message(&msg);
}

/* Feeds ArduPilot's slung-payload damping script (copter-slung-payload.lua),
 * which expects the payload to publish GLOBAL_POSITION_INT at 10Hz.
 *
 * relative_alt means "above home", and this board has no home. It used to
 * just repeat the MSL altitude there, which makes a GCS altitude tape read
 * the local ground elevation -- 843m on an inland bench -- and look broken.
 * Latching the first 3D fix and reporting the height gained since is both
 * honest and reads ~0 on the bench. alt itself stays true MSL, which is the
 * field the slung-payload script actually consumes. */
static bool s_ref_alt_valid;
static float s_ref_alt_m;

static void send_global_position_int(const nav_state_t *nav)
{
    if (!nav->gps_fix_valid) {
        return;
    }

    /* Only a 3D fix; a 2D fix has no usable altitude to anchor to. */
    if (!s_ref_alt_valid && nav->fix_type >= 3) {
        s_ref_alt_m = nav->alt_m;
        s_ref_alt_valid = true;
    }

    float course_rad = nav->course_deg * ((float)M_PI / 180.0f);
    int16_t vx = (int16_t)(nav->speed_mps * cosf(course_rad) * 100.0f); /* cm/s */
    int16_t vy = (int16_t)(nav->speed_mps * sinf(course_rad) * 100.0f);
    int16_t vz = (int16_t)(nav->vel_d_mps * 100.0f); /* cm/s, positive down */

    int32_t alt_mm = (int32_t)(nav->alt_m * 1000.0f);
    int32_t rel_alt_mm = s_ref_alt_valid
                             ? (int32_t)((nav->alt_m - s_ref_alt_m) * 1000.0f)
                             : 0;

    mavlink_message_t msg;
    mavlink_msg_global_position_int_pack(MAVLINK_LINK_SYSTEM_ID, MAVLINK_LINK_COMPONENT_ID, &msg,
                                          clock_now_ms(),
                                          nav->lat_degE7, nav->lon_degE7,
                                          alt_mm, rel_alt_mm,
                                          vx, vy, vz,
                                          heading_to_mavlink_yaw(nav->heading_valid, nav->heading_deg));
    send_message(&msg);
}

/* 1Hz bus-health diagnostic: the board has no debug console, so the MAVLink
 * link doubles as one. Shows raw CAN frames received, TX errors, the node ID
 * we handed out (0 until the DNA handshake completes), Fix2 and magnetometer
 * messages decoded, and the current compass heading (-1 = none yet). */
/* Keeps each diagnostic number to at most 4 characters so the composed
 * line provably fits STATUSTEXT's 50-byte text field. */
static int clamp999(int16_t v)
{
    if (v > 999) {
        return 999;
    }
    if (v < -999) {
        return -999;
    }
    return v;
}

static void send_dronecan_status(const nav_state_t *nav)
{
    dronecan_stats_t st;
    DroneCanGnss_GetStats(&st);

    int heading = nav->heading_valid ? (int)nav->heading_deg : -1;

    char text[50];
    if (st.tx_errors == 0) {
        snprintf(text, sizeof(text), "DC sv:%u su:%u ft:%u fx:%lu hd:%d h:%u id:%u",
                 nav->sats_visible, nav->sats_used, nav->fix_type,
                 (unsigned long)st.fix2_count, heading,
                 st.remote_health, st.allocated_id);
    } else {
        snprintf(text, sizeof(text), "DC TXERR:%lu rx:%lu id:%u fx:%lu",
                 (unsigned long)st.tx_errors, (unsigned long)st.rx_frames,
                 st.allocated_id, (unsigned long)st.fix2_count);
    }

    mavlink_message_t msg;
    mavlink_msg_statustext_pack(MAVLINK_LINK_SYSTEM_ID, MAVLINK_LINK_COMPONENT_ID, &msg,
                                 MAV_SEVERITY_INFO, text, 0, 0);
    send_message(&msg);

    /* Raw (untransformed) Here4 gyro, 0.01 rad/s: gl = rate_gyro_latest as
     * broadcast, gi = rate_gyro_integral over its integration interval.
     * Shows which of the two the peripheral actually fills. */
    snprintf(text, sizeof(text), "RAW gl:%d,%d,%d gi:%d,%d,%d",
             clamp999(st.raw_gyro[0]), clamp999(st.raw_gyro[1]),
             clamp999(st.raw_gyro[2]), clamp999(st.raw_gyro_int[0]),
             clamp999(st.raw_gyro_int[1]), clamp999(st.raw_gyro_int[2]));
    mavlink_msg_statustext_pack(MAVLINK_LINK_SYSTEM_ID, MAVLINK_LINK_COMPONENT_ID, &msg,
                                 MAV_SEVERITY_INFO, text, 0, 0);
    send_message(&msg);
}

/* Relays the Here4's own debug/error log output (DroneCAN LogMessage) so
 * problems inside the peripheral (e.g. GPS probe failures) are visible in
 * the GCS messages tab. */
static void forward_here4_log(void)
{
    char text[50];
    if (!DroneCanGnss_TakeLogMessage(text, sizeof(text))) {
        return;
    }

    mavlink_message_t msg;
    mavlink_msg_statustext_pack(MAVLINK_LINK_SYSTEM_ID, MAVLINK_LINK_COMPONENT_ID, &msg,
                                 MAV_SEVERITY_NOTICE, text, 0, 0);
    send_message(&msg);
}

/* While a DroneCAN session is up the usual bus diagnostic is suppressed
 * along with the rest of telemetry, so report on the bridge instead.
 *
 * The fields are ordered so that the ones that localise a fault survive if
 * STATUSTEXT's 50 bytes ever truncate the line:
 *
 *   e - MAVLink receive errors, i.e. bytes lost between the GCS and here.
 *       Non-zero means the serial RX path is dropping the GCS's requests,
 *       not that our forwarding is at fault.
 *   s - times the forwarding session lapsed on its 5s keepalive. Any lapse
 *       mid-download silently discards responses until the GCS asks again.
 *   t - CAN transmit errors, i.e. requests that never reached the bus.
 *   d - frames we dropped because our own forward queue was full. This is
 *       the bandwidth symptom.
 *   k - frames deliberately not forwarded: the Here4's sensor broadcasts,
 *       plus anything the GCS filtered out with CAN_FILTER_MODIFY. Counted
 *       so that path can never masquerade as loss.
 *   f - how many IDs the GCS asked us to filter down to.
 *   o/i - frames forwarded out to the GCS / in to the bus, for scale.
 *
 * The MAVLink counters are 8 and 16 bits wide inside the library, so they
 * wrap; read them as "is this moving at all", not as a total. */
static void send_can_forward_status(void)
{
    can_forward_stats_t st;
    MavlinkCanForward_GetStats(&st);

    dronecan_stats_t dc;
    DroneCanGnss_GetStats(&dc);

    mavlink_status_t *rx = mavlink_get_channel_status(MAVLINK_COMM_0);
    unsigned rx_err = (unsigned)rx->parse_error +
                      (unsigned)rx->buffer_overrun +
                      (unsigned)rx->packet_rx_drop_count;

    char text[50];
    snprintf(text, sizeof(text), "CFWD e%u s%lu t%lu d%lu k%lu f%u o%lu i%lu",
             rx_err,
             (unsigned long)st.session_lapses,
             (unsigned long)dc.tx_errors,
             (unsigned long)st.drops,
             (unsigned long)st.skipped,
             st.filter_ids,
             (unsigned long)st.frames_to_gcs,
             (unsigned long)st.frames_to_bus);

    mavlink_message_t msg;
    mavlink_msg_statustext_pack(MAVLINK_LINK_SYSTEM_ID, MAVLINK_LINK_COMPONENT_ID, &msg,
                                 MAV_SEVERITY_INFO, text, 0, 0);
    send_message(&msg);
}

void *mavlink_tx_task(void *argument)
{
    (void)argument;
    unsigned tick = 0;
    unsigned iteration = 0;

    for (;;) {
        /* The loop ticks at 100Hz because forwarded CAN frames have to move
         * far faster than telemetry does; the telemetry block below still
         * runs at its original 10Hz, on every tenth tick. */
        bool backlog = MavlinkCanForward_Drain();

        if (++tick < 10) {
            /* Sleeping while frames are still queued leaves the UART idle,
             * and idle UART time during a parameter download is a response
             * that gets dropped later. The blocking write already paces this
             * loop to the link speed, so when there is a backlog just go
             * round again. The tick still advances, so the heartbeat keeps
             * going out. */
            if (!backlog) {
                usleep(10000);
            }
            continue;
        }
        tick = 0;

        bool forwarding = MavlinkCanForward_Active();

        nav_state_t nav;
        NavState_GetSnapshot(&nav);
        imu_state_t imu;
        ImuState_GetSnapshot(&imu);
        ahrs_state_t ahrs;
        AhrsState_GetSnapshot(&ahrs);
        baro_state_t baro;
        BaroState_GetSnapshot(&baro);

        /* The heartbeat always goes out, so the GCS keeps the link up while
         * it works the DroneCAN page. */
        send_heartbeat();

        /* The rest is ~4.8KB/s, most of a 57600 link -- leaving it running
         * would starve the CAN frames Mission Planner is waiting on, and a
         * DroneCAN session is a bench activity, not a flight one. Telemetry
         * returns by itself when the session lapses; set CAN_FWD_TELEM=1 to
         * keep it running on a link with the bandwidth to spare. */
        if (!forwarding || MavlinkCanForward_GetKeepTelemetry()) {
            send_gps_input(&nav);
            send_gps_raw_int(&nav);
            send_global_position_int(&nav);
            send_highres_imu(&imu, &baro);
            send_scaled_pressure(&baro);
            send_attitude(&ahrs);
            send_vfr_hud(&nav, &ahrs);
            send_follow_target(&nav, &ahrs);

            forward_here4_log();
        }

        if (++iteration >= 10) {
            iteration = 0;
            if (forwarding) {
                send_can_forward_status();
            } else {
                send_dronecan_status(&nav);
            }
        }

        usleep(10000);
    }

    return NULL;
}

/****************************************************************************
 * Receive path
 *
 * The board ignored its RX line entirely until Mission Planner needed to
 * reach the Here4 through it. It still answers only what that needs: the
 * CAN bridge messages, and enough of the parameter protocol that Mission
 * Planner's connect sequence finishes instead of retrying forever.
 ****************************************************************************/

/* Mission Planner downloads the parameter list the moment it connects, so
 * the link has to offer one even though every other setting on this board
 * is a compile-time Kconfig choice. These two are the ones worth changing
 * without a rebuild. Neither is persisted -- this board has no storage --
 * so both revert to their defaults on reboot. */
enum {
    PARAM_CAN_FWD_ENABLE = 0,
    PARAM_CAN_FWD_TELEM,
    PARAM_CAN_FWD_BCAST,
    PARAM_COUNT
};

static const char *const g_param_names[PARAM_COUNT] = {
    "CAN_FWD_ENABLE",
    "CAN_FWD_TELEM",
    "CAN_FWD_BCAST",
};

static float param_get(unsigned index)
{
    switch (index) {
    case PARAM_CAN_FWD_ENABLE:
        return MavlinkCanForward_GetEnabled() ? 1.0f : 0.0f;
    case PARAM_CAN_FWD_TELEM:
        return MavlinkCanForward_GetKeepTelemetry() ? 1.0f : 0.0f;
    case PARAM_CAN_FWD_BCAST:
        return MavlinkCanForward_GetForwardBroadcasts() ? 1.0f : 0.0f;
    default:
        return 0.0f;
    }
}

static void param_apply(unsigned index, float value)
{
    switch (index) {
    case PARAM_CAN_FWD_ENABLE:
        MavlinkCanForward_SetEnabled(value > 0.5f);
        break;
    case PARAM_CAN_FWD_TELEM:
        MavlinkCanForward_SetKeepTelemetry(value > 0.5f);
        break;
    case PARAM_CAN_FWD_BCAST:
        MavlinkCanForward_SetForwardBroadcasts(value > 0.5f);
        break;
    default:
        break;
    }
}

static void send_param_value(unsigned index)
{
    if (index >= PARAM_COUNT) {
        return;
    }

    mavlink_message_t msg;
    mavlink_msg_param_value_pack(MAVLINK_LINK_SYSTEM_ID, MAVLINK_LINK_COMPONENT_ID, &msg,
                                  g_param_names[index], param_get(index),
                                  MAV_PARAM_TYPE_REAL32, PARAM_COUNT,
                                  (uint16_t)index);
    send_message(&msg);
}

/* param_id is not required to be NUL-terminated when it fills the field. */
static int param_index_by_name(const char *name)
{
    for (unsigned i = 0; i < PARAM_COUNT; i++) {
        if (strncmp(name, g_param_names[i], 16) == 0) {
            return (int)i;
        }
    }
    return -1;
}

/* Matched on system ID alone, deliberately. This board is the only
 * component in its system, so anything addressed to this system ID is for
 * us whichever component a GCS happens to aim at, and a mismatch would be
 * ignored in silence rather than reported. A real autopilot sharing the
 * link carries its own system ID, so its commands still do not match.
 */
static bool addressed_to_us(uint8_t target_system, uint8_t target_component)
{
    (void)target_component;
    return target_system == 0 || target_system == MAVLINK_LINK_SYSTEM_ID;
}

static void handle_command(uint16_t command, float param1,
                            uint8_t sysid, uint8_t compid)
{
    uint8_t result = MAV_RESULT_UNSUPPORTED;

    if (command == MAV_CMD_CAN_FORWARD) {
        result = MavlinkCanForward_HandleCommand(param1, sysid, compid);
    }

    mavlink_message_t msg;
    mavlink_msg_command_ack_pack(MAVLINK_LINK_SYSTEM_ID, MAVLINK_LINK_COMPONENT_ID, &msg,
                                  command, result, 0, 0, sysid, compid);
    send_message(&msg);
}

static void handle_message(const mavlink_message_t *msg)
{
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_COMMAND_LONG: {
        mavlink_command_long_t cmd;
        mavlink_msg_command_long_decode(msg, &cmd);
        if (addressed_to_us(cmd.target_system, cmd.target_component)) {
            handle_command(cmd.command, cmd.param1, msg->sysid, msg->compid);
        }
        break;
    }

    case MAVLINK_MSG_ID_COMMAND_INT: {
        mavlink_command_int_t cmd;
        mavlink_msg_command_int_decode(msg, &cmd);
        if (addressed_to_us(cmd.target_system, cmd.target_component)) {
            handle_command(cmd.command, cmd.param1, msg->sysid, msg->compid);
        }
        break;
    }

    case MAVLINK_MSG_ID_CAN_FRAME: {
        mavlink_can_frame_t frame;
        mavlink_msg_can_frame_decode(msg, &frame);
        if (addressed_to_us(frame.target_system, frame.target_component)) {
            MavlinkCanForward_HandleCanFrame(msg);
        }
        break;
    }

    case MAVLINK_MSG_ID_CAN_FILTER_MODIFY: {
        mavlink_can_filter_modify_t mod;
        mavlink_msg_can_filter_modify_decode(msg, &mod);
        if (addressed_to_us(mod.target_system, mod.target_component)) {
            MavlinkCanForward_HandleFilterModify(msg);
        }
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
        mavlink_param_request_list_t req;
        mavlink_msg_param_request_list_decode(msg, &req);
        if (addressed_to_us(req.target_system, req.target_component)) {
            for (unsigned i = 0; i < PARAM_COUNT; i++) {
                send_param_value(i);
            }
        }
        break;
    }

    case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
        mavlink_param_request_read_t req;
        mavlink_msg_param_request_read_decode(msg, &req);
        if (!addressed_to_us(req.target_system, req.target_component)) {
            break;
        }
        if (req.param_index >= 0) {
            send_param_value((unsigned)req.param_index);
        } else {
            int index = param_index_by_name(req.param_id);
            if (index >= 0) {
                send_param_value((unsigned)index);
            }
        }
        break;
    }

    case MAVLINK_MSG_ID_PARAM_SET: {
        mavlink_param_set_t set;
        mavlink_msg_param_set_decode(msg, &set);
        if (!addressed_to_us(set.target_system, set.target_component)) {
            break;
        }
        int index = param_index_by_name(set.param_id);
        if (index >= 0) {
            param_apply((unsigned)index, set.param_value);
            /* Echo the value back: that acknowledgement is what ends the
             * GCS's retry loop. */
            send_param_value((unsigned)index);
        }
        break;
    }

    default:
        break;
    }
}

void *mavlink_rx_task(void *argument)
{
    (void)argument;

    mavlink_message_t msg;
    mavlink_status_t status;

    for (;;) {
        uint8_t buf[64];

        ssize_t nread = read(s_fd, buf, sizeof(buf));
        if (nread <= 0) {
            /* A blocking serial read only lands here on error or on a
             * closed device; back off instead of spinning on it. */
            usleep(10000);
            continue;
        }

        for (ssize_t i = 0; i < nread; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                handle_message(&msg);
            }
        }
    }

    return NULL;
}
