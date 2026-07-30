/* Accel/gyro complementary filter fed by the Here4's DroneCAN RawIMU
 * broadcast (dronecan_gnss.c) -- the only IMU on the system -- with the
 * magnetometer heading fused into yaw.
 *
 * The Here4 broadcasts its IMU in a Z-down frame yawed 90 degrees from the
 * case's forward direction (verified empirically with gravity in three
 * orientations); AhrsFilter_Update() rotates it into body FRD before
 * filtering. */
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

/* Latest tilt-compensated compass heading (0-360 deg, true north CW),
 * called from the magnetometer path. Anchors the otherwise free-drifting
 * gyro yaw. */
void AhrsFilter_SetMagHeading(float heading_deg);

#ifdef __cplusplus
}
#endif

#endif /* AHRS_FILTER_H */
