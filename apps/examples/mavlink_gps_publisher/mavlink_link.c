#include "mavlink_link.h"
#include "nav_state.h"
#include "imu_state.h"
#include "ahrs_state.h"
#include "clock_ms.h"
#include "common/mavlink.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <math.h>
#include <stdint.h>

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

    uint16_t ignore_flags = GPS_INPUT_IGNORE_FLAG_VDOP |
                             GPS_INPUT_IGNORE_FLAG_VEL_VERT |
                             GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY |
                             GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY |
                             GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY;

    mavlink_message_t msg;
    mavlink_msg_gps_input_pack(MAVLINK_LINK_SYSTEM_ID, MAVLINK_LINK_COMPONENT_ID, &msg,
                                (uint64_t)clock_now_ms() * 1000ULL, /* time_usec, since-boot */
                                0,                                 /* gps_id */
                                ignore_flags,
                                0, 0,                               /* time_week_ms, time_week: not parsed from NMEA date */
                                nav->fix_type,
                                nav->lat_degE7, nav->lon_degE7,
                                nav->alt_m,
                                nav->hdop, 0.0f,                    /* hdop, vdop(ignored) */
                                vn, ve, 0.0f,                       /* vn, ve, vd(ignored) */
                                0.0f, 0.0f, 0.0f,                   /* speed/h/v accuracy (ignored) */
                                nav->satellites_visible,
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
    uint16_t eph = (nav->hdop > 0.0f) ? (uint16_t)(nav->hdop * 100.0f) : UINT16_MAX;

    mavlink_message_t msg;
    mavlink_msg_gps_raw_int_pack(MAVLINK_LINK_SYSTEM_ID, MAVLINK_LINK_COMPONENT_ID, &msg,
                                  (uint64_t)clock_now_ms() * 1000ULL,
                                  nav->fix_type,
                                  nav->lat_degE7, nav->lon_degE7,
                                  (int32_t)(nav->alt_m * 1000.0f),
                                  eph,
                                  UINT16_MAX, /* epv: vdop not available */
                                  (uint16_t)(nav->speed_mps * 100.0f),
                                  (uint16_t)(nav->course_deg * 100.0f),
                                  nav->satellites_visible,
                                  0, 0, 0, 0, 0, /* alt_ellipsoid/h_acc/v_acc/vel_acc/hdg_acc: not available */
                                  heading_to_mavlink_yaw(nav->heading_valid, nav->heading_deg));
    send_message(&msg);
}

/* Mag fields are left at 0 and unflagged: this board's MPU9250 aux-I2C
 * magnetometer (AK8963) pass-through isn't wired up yet. No barometer
 * (BMP280 CS tied high / unused), so pressure fields are omitted too. */
static void send_highres_imu(const imu_state_t *imu)
{
    if (!imu->valid) {
        return;
    }

    uint16_t fields_updated = HIGHRES_IMU_UPDATED_XACC | HIGHRES_IMU_UPDATED_YACC |
                               HIGHRES_IMU_UPDATED_ZACC | HIGHRES_IMU_UPDATED_XGYRO |
                               HIGHRES_IMU_UPDATED_YGYRO | HIGHRES_IMU_UPDATED_ZGYRO |
                               HIGHRES_IMU_UPDATED_TEMPERATURE;

    mavlink_message_t msg;
    mavlink_msg_highres_imu_pack(MAVLINK_LINK_SYSTEM_ID, MAVLINK_LINK_COMPONENT_ID, &msg,
                                  (uint64_t)clock_now_ms() * 1000ULL,
                                  imu->accel_x_mps2, imu->accel_y_mps2, imu->accel_z_mps2,
                                  imu->gyro_x_rads, imu->gyro_y_rads, imu->gyro_z_rads,
                                  0.0f, 0.0f, 0.0f,             /* xmag/ymag/zmag: not available */
                                  0.0f, 0.0f, 0.0f,             /* abs/diff pressure, pressure_alt: not available */
                                  imu->temperature_degc,
                                  fields_updated, 0);
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
    float vel[3] = { nav->speed_mps * cosf(course_rad), nav->speed_mps * sinf(course_rad), 0.0f };
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

/* Drives the GCS heading tape / speed / altitude tapes. airspeed/throttle/climb
 * are 0: no pitot, no ESC telemetry, no vertical-speed source on this board. */
static void send_vfr_hud(const nav_state_t *nav, const ahrs_state_t *ahrs)
{
    int16_t heading_deg = ahrs->valid ? rad_to_heading_deg(ahrs->yaw_rad) : 0;

    mavlink_message_t msg;
    mavlink_msg_vfr_hud_pack(MAVLINK_LINK_SYSTEM_ID, MAVLINK_LINK_COMPONENT_ID, &msg,
                              0.0f, nav->speed_mps, heading_deg, 0, nav->alt_m, 0.0f);
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

    int32_t alt_mm = (int32_t)(nav->alt_m * 1000.0f);

    mavlink_message_t msg;
    mavlink_msg_global_position_int_pack(MAVLINK_LINK_SYSTEM_ID, MAVLINK_LINK_COMPONENT_ID, &msg,
                                          clock_now_ms(),
                                          nav->lat_degE7, nav->lon_degE7,
                                          alt_mm, alt_mm,
                                          vx, vy, 0,
                                          heading_to_mavlink_yaw(nav->heading_valid, nav->heading_deg));
    send_message(&msg);
}

void *mavlink_tx_task(void *argument)
{
    (void)argument;
    for (;;) {
        nav_state_t nav;
        NavState_GetSnapshot(&nav);
        imu_state_t imu;
        ImuState_GetSnapshot(&imu);
        ahrs_state_t ahrs;
        AhrsState_GetSnapshot(&ahrs);

        send_heartbeat();
        send_gps_input(&nav);
        send_gps_raw_int(&nav);
        send_global_position_int(&nav);
        send_highres_imu(&imu);
        send_attitude(&ahrs);
        send_vfr_hud(&nav, &ahrs);
        send_follow_target(&nav, &ahrs);

        usleep(100000); /* 10Hz, matching upstream follow-target-send.lua's UPDATE_INTERVAL_MS */
    }

    return NULL;
}
