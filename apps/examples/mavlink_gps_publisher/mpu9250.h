/* MPU9250 accel/gyro driver over SPI1 (GY-91 module, BMP280 and AK8963
 * magnetometer not used). CS is PA4, driven by board logic via
 * stm32_spi1select() (software NSS management). */
#ifndef MPU9250_H
#define MPU9250_H

#include <stdbool.h>
#include <nuttx/spi/spi.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Initializes the sensor (reset, wake, +-8g/+-500dps, DLPF) over the given,
 * already-initialized SPI bus. Returns false if WHO_AM_I didn't match a
 * known MPU9250/MPU9255 id. */
bool MPU9250_Init(struct spi_dev_s *spi);

/* pthread entry: polls the sensor and pushes accel/gyro into imu_state. */
void *imu_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* MPU9250_H */
