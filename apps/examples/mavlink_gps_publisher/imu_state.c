#include "imu_state.h"
#include "clock_ms.h"
#include <pthread.h>

static imu_state_t s_state;
static pthread_mutex_t s_mutex;
static uint32_t s_imu_last_tick;

void ImuState_Init(void)
{
    pthread_mutex_init(&s_mutex, NULL);
    s_state = (imu_state_t){0};
    s_imu_last_tick = clock_now_ms();
}

void ImuState_Update(float accel_x_mps2, float accel_y_mps2, float accel_z_mps2,
                      float gyro_x_rads, float gyro_y_rads, float gyro_z_rads)
{
    pthread_mutex_lock(&s_mutex);
    s_state.valid = true;
    s_state.accel_x_mps2 = accel_x_mps2;
    s_state.accel_y_mps2 = accel_y_mps2;
    s_state.accel_z_mps2 = accel_z_mps2;
    s_state.gyro_x_rads = gyro_x_rads;
    s_state.gyro_y_rads = gyro_y_rads;
    s_state.gyro_z_rads = gyro_z_rads;
    s_imu_last_tick = clock_now_ms();
    pthread_mutex_unlock(&s_mutex);
}

void ImuState_GetSnapshot(imu_state_t *out)
{
    pthread_mutex_lock(&s_mutex);
    *out = s_state;
    out->imu_age_ms = clock_now_ms() - s_imu_last_tick;
    pthread_mutex_unlock(&s_mutex);
}
