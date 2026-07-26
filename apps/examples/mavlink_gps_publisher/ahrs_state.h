/* Shared, mutex-protected attitude estimate written by imu_task()'s
 * complementary filter and read by mavlink_tx_task(). */
#ifndef AHRS_STATE_H
#define AHRS_STATE_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool valid;
    float roll_rad;          /* -pi..+pi, accel+gyro complementary filter */
    float pitch_rad;         /* -pi..+pi, accel+gyro complementary filter */
    float yaw_rad;           /* -pi..+pi, free-integrated from gyro only:
                               * no magnetometer, so this drifts over time. */
    float rollspeed_rads;
    float pitchspeed_rads;
    float yawspeed_rads;
} ahrs_state_t;

void AhrsState_Init(void);

void AhrsState_Update(float roll_rad, float pitch_rad, float yaw_rad,
                       float rollspeed_rads, float pitchspeed_rads, float yawspeed_rads);

void AhrsState_GetSnapshot(ahrs_state_t *out);

#ifdef __cplusplus
}
#endif

#endif /* AHRS_STATE_H */
