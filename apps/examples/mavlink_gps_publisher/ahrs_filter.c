#include "ahrs_filter.h"
#include "ahrs_state.h"
#include "imu_state.h"
#include "clock_ms.h"

#include <math.h>
#include <pthread.h>
#include <stdbool.h>

/* Complementary filter weight on gyro-integrated angle vs. the accelerometer's
 * gravity-vector estimate; 0.98 trusts the gyro short-term and lets the
 * (noisier but drift-free) accel reading correct it slowly. Assumes the
 * source is mounted/oriented X-forward, Y-right, Z-down (MAVLink body
 * frame) -- true for the onboard MPU9250, and the DroneCAN RawIMU
 * convention matches it too. */
#define AHRS_COMPLEMENTARY_ALPHA 0.98f

/* A CAN (Here4) sample is treated as "alive" -- and takes priority over the
 * onboard MPU9250 -- for this long after its last update, mirroring
 * nav_state's GPS priority window. */
#define AHRS_CAN_PRIORITY_WINDOW_MS 2000U

static pthread_mutex_t s_mutex;
static bool s_initialized;
static float s_roll_rad;
static float s_pitch_rad;
static float s_yaw_rad;
static uint32_t s_can_last_tick;
static uint32_t s_last_fused_tick;

void AhrsFilter_Init(void)
{
    pthread_mutex_init(&s_mutex, NULL);
    s_initialized = false;
    s_roll_rad = 0.0f;
    s_pitch_rad = 0.0f;
    s_yaw_rad = 0.0f;
    s_can_last_tick = 0U;
    s_last_fused_tick = 0U;
    pthread_mutex_unlock(&s_mutex);
}

void AhrsFilter_Update(float accel_x_mps2, float accel_y_mps2, float accel_z_mps2,
                        float gyro_x_rads, float gyro_y_rads, float gyro_z_rads,
                        float temperature_degc, ahrs_source_t source)
{
    pthread_mutex_lock(&s_mutex);

    uint32_t now = clock_now_ms();
    if (source == AHRS_SOURCE_MPU9250 &&
        (now - s_can_last_tick) < AHRS_CAN_PRIORITY_WINDOW_MS) {
        /* A CAN (Here4) sample is still active: ignore the MPU9250 sample. */
        pthread_mutex_unlock(&s_mutex);
        return;
    }
    if (source == AHRS_SOURCE_CAN) {
        s_can_last_tick = now;
    }

    float dt_s = (s_last_fused_tick == 0U) ? 0.02f : (float)(now - s_last_fused_tick) / 1000.0f;
    if (dt_s <= 0.0f) {
        dt_s = 0.02f; /* first sample / tick didn't advance: assume nominal period */
    }
    s_last_fused_tick = now;

    /* Roll/pitch from the accel+gyro complementary filter; yaw is
     * free-integrated from the gyro alone (no magnetometer feeds this
     * filter -- Here4's compass drives nav_state's heading separately via
     * dronecan_gnss.c), so it will drift over time. */
    float roll_acc = atan2f(accel_y_mps2, accel_z_mps2);
    float pitch_acc = atan2f(-accel_x_mps2, sqrtf(accel_y_mps2 * accel_y_mps2 + accel_z_mps2 * accel_z_mps2));

    if (!s_initialized) {
        s_roll_rad = roll_acc;
        s_pitch_rad = pitch_acc;
        s_yaw_rad = 0.0f;
        s_initialized = true;
    } else {
        s_roll_rad = AHRS_COMPLEMENTARY_ALPHA * (s_roll_rad + gyro_x_rads * dt_s) +
                     (1.0f - AHRS_COMPLEMENTARY_ALPHA) * roll_acc;
        s_pitch_rad = AHRS_COMPLEMENTARY_ALPHA * (s_pitch_rad + gyro_y_rads * dt_s) +
                      (1.0f - AHRS_COMPLEMENTARY_ALPHA) * pitch_acc;
        s_yaw_rad += gyro_z_rads * dt_s;
        if (s_yaw_rad > (float)M_PI) {
            s_yaw_rad -= 2.0f * (float)M_PI;
        } else if (s_yaw_rad < -(float)M_PI) {
            s_yaw_rad += 2.0f * (float)M_PI;
        }
    }

    AhrsState_Update(s_roll_rad, s_pitch_rad, s_yaw_rad,
                      gyro_x_rads, gyro_y_rads, gyro_z_rads);
    ImuState_Update(accel_x_mps2, accel_y_mps2, accel_z_mps2,
                     gyro_x_rads, gyro_y_rads, gyro_z_rads,
                     temperature_degc);

    pthread_mutex_unlock(&s_mutex);
}
