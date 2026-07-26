#include "mpu9250.h"
#include "ahrs_filter.h"
#include <stdint.h>
#include <math.h>
#include <unistd.h>

/* MPU9250 register map (subset actually used). */
#define MPU9250_REG_SMPLRT_DIV      0x19U
#define MPU9250_REG_CONFIG          0x1AU
#define MPU9250_REG_GYRO_CONFIG     0x1BU
#define MPU9250_REG_ACCEL_CONFIG    0x1CU
#define MPU9250_REG_ACCEL_CONFIG2   0x1DU
#define MPU9250_REG_USER_CTRL       0x6AU
#define MPU9250_REG_PWR_MGMT_1      0x6BU
#define MPU9250_REG_PWR_MGMT_2      0x6CU
#define MPU9250_REG_WHO_AM_I        0x75U
#define MPU9250_REG_ACCEL_XOUT_H    0x3BU

#define MPU9250_SPI_READ_BIT        0x80U

#define MPU9250_PWR1_H_RESET        0x80U
#define MPU9250_PWR1_CLKSEL_PLL     0x01U
#define MPU9250_USER_CTRL_I2C_IF_DIS 0x10U

/* GYRO_CONFIG FS_SEL=01 -> +-500 dps; ACCEL_CONFIG AFS_SEL=10 -> +-8g. */
#define MPU9250_GYRO_CONFIG_500DPS   0x08U
#define MPU9250_ACCEL_CONFIG_8G      0x10U
#define MPU9250_ACCEL_CONFIG2_DLPF44 0x03U
#define MPU9250_CONFIG_DLPF41        0x03U
#define MPU9250_SMPLRT_DIV_200HZ     0x04U

#define MPU9250_ACCEL_LSB_PER_G      4096.0f  /* +-8g full scale */
#define MPU9250_GYRO_LSB_PER_DPS     65.5f    /* +-500 dps full scale */
#define STANDARD_GRAVITY             9.80665f
#define DEG_TO_RAD                   ((float)M_PI / 180.0f)

/* Comfortably under the MPU9250's 1 MHz register-access limit. */
#define MPU9250_SPI_FREQUENCY        562500U

static struct spi_dev_s *s_spi;

static void mpu9250_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t txbuf[2] = { (uint8_t)(reg & 0x7FU), value };
    uint8_t rxbuf[2];

    SPI_LOCK(s_spi, true);
    SPI_SETMODE(s_spi, SPIDEV_MODE0);
    SPI_SETBITS(s_spi, 8);
    SPI_SETFREQUENCY(s_spi, MPU9250_SPI_FREQUENCY);
    SPI_SELECT(s_spi, SPIDEV_IMU(0), true);
    SPI_EXCHANGE(s_spi, txbuf, rxbuf, sizeof(txbuf));
    SPI_SELECT(s_spi, SPIDEV_IMU(0), false);
    SPI_LOCK(s_spi, false);
}

static uint8_t mpu9250_read_reg(uint8_t reg)
{
    uint8_t txbuf[2] = { (uint8_t)(reg | MPU9250_SPI_READ_BIT), 0x00U };
    uint8_t rxbuf[2];

    SPI_LOCK(s_spi, true);
    SPI_SETMODE(s_spi, SPIDEV_MODE0);
    SPI_SETBITS(s_spi, 8);
    SPI_SETFREQUENCY(s_spi, MPU9250_SPI_FREQUENCY);
    SPI_SELECT(s_spi, SPIDEV_IMU(0), true);
    SPI_EXCHANGE(s_spi, txbuf, rxbuf, sizeof(txbuf));
    SPI_SELECT(s_spi, SPIDEV_IMU(0), false);
    SPI_LOCK(s_spi, false);

    return rxbuf[1];
}

/* Sends reg|READ_BIT followed by len dummy bytes in one full-duplex
 * transfer; the MPU9250 auto-increments its internal register pointer
 * on each byte for a burst read. */
static void mpu9250_read_burst(uint8_t start_reg, uint8_t *buf, uint16_t len)
{
    uint8_t txbuf[15];
    uint8_t rxbuf[15];

    txbuf[0] = (uint8_t)(start_reg | MPU9250_SPI_READ_BIT);
    for (uint16_t i = 0; i < len; i++) {
        txbuf[1 + i] = 0x00U;
    }

    SPI_LOCK(s_spi, true);
    SPI_SETMODE(s_spi, SPIDEV_MODE0);
    SPI_SETBITS(s_spi, 8);
    SPI_SETFREQUENCY(s_spi, MPU9250_SPI_FREQUENCY);
    SPI_SELECT(s_spi, SPIDEV_IMU(0), true);
    SPI_EXCHANGE(s_spi, txbuf, rxbuf, len + 1U);
    SPI_SELECT(s_spi, SPIDEV_IMU(0), false);
    SPI_LOCK(s_spi, false);

    for (uint16_t i = 0; i < len; i++) {
        buf[i] = rxbuf[1 + i];
    }
}

bool MPU9250_Init(struct spi_dev_s *spi)
{
    s_spi = spi;

    mpu9250_write_reg(MPU9250_REG_PWR_MGMT_1, MPU9250_PWR1_H_RESET);
    usleep(100000); /* datasheet: allow >=100ms after H_RESET */

    mpu9250_write_reg(MPU9250_REG_USER_CTRL, MPU9250_USER_CTRL_I2C_IF_DIS);
    mpu9250_write_reg(MPU9250_REG_PWR_MGMT_1, MPU9250_PWR1_CLKSEL_PLL);
    usleep(10000); /* let the PLL settle before trusting gyro output */

    mpu9250_write_reg(MPU9250_REG_PWR_MGMT_2, 0x00U); /* enable all accel + gyro axes */
    mpu9250_write_reg(MPU9250_REG_CONFIG, MPU9250_CONFIG_DLPF41);
    mpu9250_write_reg(MPU9250_REG_SMPLRT_DIV, MPU9250_SMPLRT_DIV_200HZ);
    mpu9250_write_reg(MPU9250_REG_GYRO_CONFIG, MPU9250_GYRO_CONFIG_500DPS);
    mpu9250_write_reg(MPU9250_REG_ACCEL_CONFIG, MPU9250_ACCEL_CONFIG_8G);
    mpu9250_write_reg(MPU9250_REG_ACCEL_CONFIG2, MPU9250_ACCEL_CONFIG2_DLPF44);

    uint8_t who_am_i = mpu9250_read_reg(MPU9250_REG_WHO_AM_I);
    /* Genuine MPU9250 = 0x71; MPU9255/clone silicon seen on GY-91 boards
     * reports 0x73 or 0x70. */
    return (who_am_i == 0x71U) || (who_am_i == 0x73U) || (who_am_i == 0x70U);
}

void *imu_task(void *argument)
{
    (void)argument;
    for (;;) {
        uint8_t raw[14];
        mpu9250_read_burst(MPU9250_REG_ACCEL_XOUT_H, raw, sizeof(raw));

        int16_t accel_x_raw = (int16_t)((raw[0] << 8) | raw[1]);
        int16_t accel_y_raw = (int16_t)((raw[2] << 8) | raw[3]);
        int16_t accel_z_raw = (int16_t)((raw[4] << 8) | raw[5]);
        int16_t temp_raw    = (int16_t)((raw[6] << 8) | raw[7]);
        int16_t gyro_x_raw  = (int16_t)((raw[8] << 8) | raw[9]);
        int16_t gyro_y_raw  = (int16_t)((raw[10] << 8) | raw[11]);
        int16_t gyro_z_raw  = (int16_t)((raw[12] << 8) | raw[13]);

        float accel_x_mps2 = ((float)accel_x_raw / MPU9250_ACCEL_LSB_PER_G) * STANDARD_GRAVITY;
        float accel_y_mps2 = ((float)accel_y_raw / MPU9250_ACCEL_LSB_PER_G) * STANDARD_GRAVITY;
        float accel_z_mps2 = ((float)accel_z_raw / MPU9250_ACCEL_LSB_PER_G) * STANDARD_GRAVITY;

        float gyro_x_rads = ((float)gyro_x_raw / MPU9250_GYRO_LSB_PER_DPS) * DEG_TO_RAD;
        float gyro_y_rads = ((float)gyro_y_raw / MPU9250_GYRO_LSB_PER_DPS) * DEG_TO_RAD;
        float gyro_z_rads = ((float)gyro_z_raw / MPU9250_GYRO_LSB_PER_DPS) * DEG_TO_RAD;

        float temperature_degc = ((float)temp_raw / 333.87f) + 21.0f;

        AhrsFilter_Update(accel_x_mps2, accel_y_mps2, accel_z_mps2,
                          gyro_x_rads, gyro_y_rads, gyro_z_rads,
                          temperature_degc, AHRS_SOURCE_MPU9250);

        usleep(20000); /* ~50 Hz */
    }

    return NULL;
}
