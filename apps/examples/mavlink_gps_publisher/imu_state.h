/* Shared, mutex-protected accel/gyro state written by dronecan_task()
 * (Here4 RawIMU broadcasts) and read by mavlink_tx_task(). */
#ifndef IMU_STATE_H
#define IMU_STATE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool valid;
    float accel_x_mps2;
    float accel_y_mps2;
    float accel_z_mps2;
    float gyro_x_rads;
    float gyro_y_rads;
    float gyro_z_rads;
    uint32_t imu_age_ms;      /* time since last update, filled by GetSnapshot */
} imu_state_t;

void ImuState_Init(void);

void ImuState_Update(float accel_x_mps2, float accel_y_mps2, float accel_z_mps2,
                      float gyro_x_rads, float gyro_y_rads, float gyro_z_rads);

/* Copies out the current state; imu_age_ms is computed relative to now. */
void ImuState_GetSnapshot(imu_state_t *out);

#ifdef __cplusplus
}
#endif

#endif /* IMU_STATE_H */
