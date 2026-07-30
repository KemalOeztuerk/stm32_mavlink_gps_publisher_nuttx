/* Accel/gyro complementary filter fed by the Here4's DroneCAN RawIMU
 * broadcast (dronecan_gnss.c) -- the only IMU on the system. Keeps a single
 * continuous roll/pitch/yaw estimate and a single dt clock. */
#ifndef AHRS_FILTER_H
#define AHRS_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

void AhrsFilter_Init(void);

/* Feeds one accel/gyro sample into the filter and updates ahrs_state and
 * imu_state. */
void AhrsFilter_Update(float accel_x_mps2, float accel_y_mps2, float accel_z_mps2,
                        float gyro_x_rads, float gyro_y_rads, float gyro_z_rads);

#ifdef __cplusplus
}
#endif

#endif /* AHRS_FILTER_H */
