#include "mavlink_link.h"
#include "nav_state.h"
#include "imu_state.h"
#include "ahrs_state.h"
#include "baro_state.h"
#include "dronecan_gnss.h"
#include "clock_ms.h"
#include "common/mavlink.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

static int s_fd = -1;

int MavlinkLink_Init(const char *devpath)
{
    int fd = open(devpath, O_RDWR);
    if (fd < 0) {
        return -1;
    }

    struct termios tio;
    if (tcgetattr(fd, &tio) == 0) {
        cfsetspeed(&tio, 57600);
        tio.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
        tio.c_cflag |= CS8;
        tcsetattr(fd, TCSANOW, &tio);
    }

    s_fd = fd;
    return 0;
}

static void send_message(const mavlink_message_t *msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    write(s_fd, buf, len);
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
    mavlink_msg_heartbeat_pack(MAVLINK_LINK_SYSTEM_ID, MAVLINK_LINK_COMPONENT_ID, &msg,
                                MAV_TYPE_GPS, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);
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
 * down velocity. */
static void send_vfr_hud(const nav_state_t *nav, const ahrs_state_t *ahrs)
{
    int16_t heading_deg = ahrs->valid ? rad_to_heading_deg(ahrs->yaw_rad) : 0;

    mavlink_message_t msg;
    mavlink_msg_vfr_hud_pack(MAVLINK_LINK_SYSTEM_ID, MAVLINK_LINK_COMPONENT_ID, &msg,
                              0.0f, nav->speed_mps, heading_deg, 0, nav->alt_m,
                              -nav->vel_d_mps);
    send_message(&msg);
}

/* Feeds ArduPilot's slung-payload damping script (copter-slung-payload.lua),
 * which expects the payload to publish GLOBAL_POSITION_INT at 10Hz.
 * relative_alt is set equal to alt: this board has no home-position
 * reference, so there's no meaningful "relative to home" altitude to
 * report. */
static void send_global_position_int(const nav_state_t *nav)
{
    if (!nav->gps_fix_valid) {
        return;
    }

    float course_rad = nav->course_deg * ((float)M_PI / 180.0f);
    int16_t vx = (int16_t)(nav->speed_mps * cosf(course_rad) * 100.0f); /* cm/s */
    int16_t vy = (int16_t)(nav->speed_mps * sinf(course_rad) * 100.0f);
    int16_t vz = (int16_t)(nav->vel_d_mps * 100.0f); /* cm/s, positive down */

    int32_t alt_mm = (int32_t)(nav->alt_m * 1000.0f);

    mavlink_message_t msg;
    mavlink_msg_global_position_int_pack(MAVLINK_LINK_SYSTEM_ID, MAVLINK_LINK_COMPONENT_ID, &msg,
                                          clock_now_ms(),
                                          nav->lat_degE7, nav->lon_degE7,
                                          alt_mm, alt_mm,
                                          vx, vy, vz,
                                          heading_to_mavlink_yaw(nav->heading_valid, nav->heading_deg));
    send_message(&msg);
}

/* 1Hz bus-health diagnostic: the board has no debug console, so the MAVLink
 * link doubles as one. Shows raw CAN frames received, TX errors, the node ID
 * we handed out (0 until the DNA handshake completes), Fix2 and magnetometer
 * messages decoded, and the current compass heading (-1 = none yet). */
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

    /* Raw (untransformed) Here4 accelerometer, 0.1 m/s^2 per LSB -- for
     * pinning down the IMU frame orientation against physical tilts. */
    snprintf(text, sizeof(text), "RAW ax:%d ay:%d az:%d",
             st.raw_acc[0], st.raw_acc[1], st.raw_acc[2]);
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

void *mavlink_tx_task(void *argument)
{
    (void)argument;
    unsigned iteration = 0;
    for (;;) {
        nav_state_t nav;
        NavState_GetSnapshot(&nav);
        imu_state_t imu;
        ImuState_GetSnapshot(&imu);
        ahrs_state_t ahrs;
        AhrsState_GetSnapshot(&ahrs);
        baro_state_t baro;
        BaroState_GetSnapshot(&baro);

        send_heartbeat();
        send_gps_input(&nav);
        send_gps_raw_int(&nav);
        send_global_position_int(&nav);
        send_highres_imu(&imu, &baro);
        send_scaled_pressure(&baro);
        send_attitude(&ahrs);
        send_vfr_hud(&nav, &ahrs);
        send_follow_target(&nav, &ahrs);

        forward_here4_log();
        if (++iteration >= 10) {
            iteration = 0;
            send_dronecan_status(&nav);
        }

        usleep(100000); /* 10Hz, matching upstream follow-target-send.lua's UPDATE_INTERVAL_MS */
    }

    return NULL;
}
