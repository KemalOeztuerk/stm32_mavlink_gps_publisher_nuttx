/* Shared accel/gyro complementary filter, fed by whichever IMU source is
 * currently authoritative: the onboard MPU9250 (mpu9250.c) or the Here4's
 * DroneCAN RawIMU broadcast (dronecan_gnss.c). AHRS_SOURCE_CAN takes
 * priority over AHRS_SOURCE_MPU9250 for as long as it keeps updating,
 * mirroring nav_state's GPS priority/fallback -- see AhrsFilter_Update().
 *
 * Centralizing this (rather than each source running its own filter
 * instance) keeps a single continuous roll/pitch/yaw estimate and a single
 * dt clock across source handoffs, instead of two filters drifting
 * independently and a hard cut on switchover.
 */
#ifndef AHRS_FILTER_H
#define AHRS_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    AHRS_SOURCE_MPU9250,
    AHRS_SOURCE_CAN,
} ahrs_source_t;

void AhrsFilter_Init(void);

/* Feeds one accel/gyro sample into the filter and updates ahrs_state (and
 * imu_state, with temperature_degc passed through as given -- RawIMU has no
 * temperature field, so callers without one should pass through the last
 * known value rather than a fabricated 0.0f). Samples from
 * AHRS_SOURCE_MPU9250 are silently dropped while a CAN sample has arrived
 * within the last couple of seconds. */
void AhrsFilter_Update(float accel_x_mps2, float accel_y_mps2, float accel_z_mps2,
                        float gyro_x_rads, float gyro_y_rads, float gyro_z_rads,
                        float temperature_degc, ahrs_source_t source);

#ifdef __cplusplus
}
#endif

#endif /* AHRS_FILTER_H */
