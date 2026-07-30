#include "ahrs_filter.h"
#include "ahrs_state.h"
#include "imu_state.h"
#include "clock_ms.h"

#include <math.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>

/* Complementary filter weight on gyro-integrated angle vs. the reference
 * (accelerometer gravity vector for roll/pitch, magnetometer heading for
 * yaw); 0.98 trusts the gyro short-term and lets the noisier but drift-free
 * reference correct it slowly. */
#define AHRS_COMPLEMENTARY_ALPHA 0.98f

#define GRAVITY_MPS2 9.80665f

/* Accel samples are only used as a gravity reference while the measured
 * specific force is close to 1g -- during shaking/swinging (slung payload!)
 * the accelerometer measures motion, not gravity, and would corrupt the
 * attitude. */
#define ACCEL_TRUST_BAND_MPS2 3.0f

/* Mag heading is used to correct yaw only while reasonably fresh. */
#define MAG_FRESH_MS 1000U

static pthread_mutex_t s_mutex;
static bool s_initialized;
static float s_roll_rad;
static float s_pitch_rad;
static float s_yaw_rad;
static bool s_yaw_anchored;      /* yaw has been set from the compass */
static uint32_t s_last_fused_tick;

static float s_mag_yaw_rad;
static uint32_t s_mag_tick;
static bool s_mag_seen;

static float wrap_pi(float a)
{
    while (a > (float)M_PI) {
        a -= 2.0f * (float)M_PI;
    }
    while (a < -(float)M_PI) {
        a += 2.0f * (float)M_PI;
    }
    return a;
}

void AhrsFilter_Init(void)
{
    pthread_mutex_init(&s_mutex, NULL);
    s_initialized = false;
    s_roll_rad = 0.0f;
    s_pitch_rad = 0.0f;
    s_yaw_rad = 0.0f;
    s_yaw_anchored = false;
    s_last_fused_tick = 0U;
    s_mag_seen = false;
}

void AhrsFilter_SetMagHeading(float heading_deg)
{
    pthread_mutex_lock(&s_mutex);
    s_mag_yaw_rad = wrap_pi(heading_deg * ((float)M_PI / 180.0f));
    s_mag_tick = clock_now_ms();
    s_mag_seen = true;
    pthread_mutex_unlock(&s_mutex);
}

void AhrsFilter_Update(float accel_x_mps2, float accel_y_mps2, float accel_z_mps2,
                        float gyro_x_rads, float gyro_y_rads, float gyro_z_rads)
{
    pthread_mutex_lock(&s_mutex);

    uint32_t now = clock_now_ms();
    float norm = sqrtf(accel_x_mps2 * accel_x_mps2 +
                       accel_y_mps2 * accel_y_mps2 +
                       accel_z_mps2 * accel_z_mps2);

    /* Here4 RawIMU frame -> body FRD. Verified empirically against a real
     * Here4 with gravity in three orientations: level reads -g on Z (Z-down
     * frame, good), but nose-down puts -g on sensor Y and right-edge-down
     * puts +g on sensor X -- i.e. the IMU frame is yawed 90 degrees from
     * the case's forward. Pure yaw rotation: x_b = y_s, y_b = -x_s. */
    float ax = accel_y_mps2;
    float ay = -accel_x_mps2;
    float az = accel_z_mps2;
    float gx = gyro_y_rads;
    float gy = -gyro_x_rads;
    float gz = gyro_z_rads;

    float dt_s = (s_last_fused_tick == 0U) ? 0.02f : (float)(now - s_last_fused_tick) / 1000.0f;
    if (dt_s <= 0.0f || dt_s > 0.5f) {
        dt_s = 0.02f; /* first sample / stalled stream: assume nominal period */
    }
    s_last_fused_tick = now;

    /* FRD specific force: level is (0, 0, -g), so gravity in the body frame
     * is -f and roll/pitch follow the standard aviation formulas. Trust the
     * accelerometer only near 1g (see ACCEL_TRUST_BAND_MPS2). */
    bool accel_ok = fabsf(norm - GRAVITY_MPS2) < ACCEL_TRUST_BAND_MPS2;
    float roll_acc = atan2f(-ay, -az);
    float pitch_acc = atan2f(ax, sqrtf(ay * ay + az * az));

    if (!s_initialized) {
        if (!accel_ok) {
            pthread_mutex_unlock(&s_mutex);
            return; /* wait for a quiet sample to initialize from */
        }
        s_roll_rad = roll_acc;
        s_pitch_rad = pitch_acc;
        s_initialized = true;
    } else {
        float roll_gyro = s_roll_rad + gx * dt_s;
        float pitch_gyro = s_pitch_rad + gy * dt_s;
        if (accel_ok) {
            s_roll_rad = AHRS_COMPLEMENTARY_ALPHA * roll_gyro +
                         (1.0f - AHRS_COMPLEMENTARY_ALPHA) * roll_acc;
            s_pitch_rad = AHRS_COMPLEMENTARY_ALPHA * pitch_gyro +
                          (1.0f - AHRS_COMPLEMENTARY_ALPHA) * pitch_acc;
        } else {
            s_roll_rad = roll_gyro;
            s_pitch_rad = pitch_gyro;
        }
    }

    /* Yaw: integrate the gyro, anchored to the tilt-compensated compass
     * heading whenever one is fresh. The first heading snaps yaw directly
     * so it is meaningful immediately instead of starting at an arbitrary
     * zero. */
    s_yaw_rad = wrap_pi(s_yaw_rad + gz * dt_s);
    if (s_mag_seen && (now - s_mag_tick) < MAG_FRESH_MS) {
        if (!s_yaw_anchored) {
            s_yaw_rad = s_mag_yaw_rad;
            s_yaw_anchored = true;
        } else {
            float err = wrap_pi(s_mag_yaw_rad - s_yaw_rad);
            s_yaw_rad = wrap_pi(s_yaw_rad + (1.0f - AHRS_COMPLEMENTARY_ALPHA) * err);
        }
    }

    /* Roll (and roll rate, to stay consistent) is published negated:
     * bench-verified against this unit's mounting. If pitch ever reads
     * reversed too, the real cause is a 180-degree forward reference flip
     * -- fix the frame mapping above instead of adding another negation. */
    AhrsState_Update(-s_roll_rad, s_pitch_rad, s_yaw_rad, -gx, gy, gz);
    ImuState_Update(ax, ay, az, gx, gy, gz);

    pthread_mutex_unlock(&s_mutex);
}
