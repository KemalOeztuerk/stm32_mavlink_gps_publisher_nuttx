/****************************************************************************
 * apps/examples/mavlink_gps_publisher/mavlink_gps_publisher_main.c
 *
 * Wires together the GPS NMEA parser, MPU9250 IMU driver, complementary-
 * filter AHRS, and MAVLink telemetry publisher, each running as its own
 * pthread -- the NuttX equivalent of the original firmware's three FreeRTOS
 * tasks (GPSTask, ImuTask, MavlinkTxTask) started from main.c.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <pthread.h>
#include <syslog.h>
#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "nav_state.h"
#include "imu_state.h"
#include "ahrs_state.h"
#include "ahrs_filter.h"
#include "gps_nmea.h"
#include "mavlink_link.h"
#include "mpu9250.h"
#include "dronecan_gnss.h"

#define GPS_TASK_STACKSIZE      2048
#define IMU_TASK_STACKSIZE      4096
#define MAVLINK_TX_STACKSIZE    4096
#define DRONECAN_TASK_STACKSIZE 3072

static void start_thread(pthread_t *thread, void *(*entry)(void *),
                          size_t stacksize, const char *name)
{
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, stacksize);

    int ret = pthread_create(thread, &attr, entry, NULL);
    if (ret != 0) {
        syslog(LOG_ERR, "mavlink_gps_publisher: failed to start %s: %d\n",
               name, ret);
    }
}

int main(int argc, FAR char *argv[])
{
    NavState_Init();
    ImuState_Init();
    AhrsState_Init();
    AhrsFilter_Init();

    if (GPSNMEA_Init(CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER_GPS_DEVPATH) < 0) {
        syslog(LOG_ERR, "mavlink_gps_publisher: failed to open GPS device %s\n",
               CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER_GPS_DEVPATH);
        return -1;
    }

    if (MavlinkLink_Init(CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER_MAVLINK_DEVPATH) < 0) {
        syslog(LOG_ERR, "mavlink_gps_publisher: failed to open MAVLink device %s\n",
               CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER_MAVLINK_DEVPATH);
        return -1;
    }

    struct spi_dev_s *spi = board_mpu9250_spibus();
    if (spi == NULL) {
        syslog(LOG_ERR, "mavlink_gps_publisher: failed to get SPI%d bus\n",
               CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER_SPI_DEVPATH);
        return -1;
    }

    if (!MPU9250_Init(spi)) {
        syslog(LOG_WARNING, "mavlink_gps_publisher: MPU9250 WHO_AM_I mismatch, "
               "continuing anyway\n");
    }

    /* Here4/DroneCAN is an optional augmentation of the NMEA GPS path (see
     * nav_state's CAN-priority fallback), so a failure here is not fatal --
     * the NMEA GPS keeps working on its own. */
    bool have_dronecan =
        (DroneCanGnss_Init(CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER_CAN_DEVPATH) == 0);
    if (!have_dronecan) {
        syslog(LOG_WARNING, "mavlink_gps_publisher: failed to open CAN device %s, "
               "Here4/DroneCAN GNSS disabled\n",
               CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER_CAN_DEVPATH);
    }

    pthread_t gps_thread;
    pthread_t imu_thread;
    pthread_t mavlink_thread;
    pthread_t dronecan_thread;

    start_thread(&gps_thread, gps_task, GPS_TASK_STACKSIZE, "gps_task");
    start_thread(&imu_thread, imu_task, IMU_TASK_STACKSIZE, "imu_task");
    start_thread(&mavlink_thread, mavlink_tx_task, MAVLINK_TX_STACKSIZE,
                 "mavlink_tx_task");
    if (have_dronecan) {
        start_thread(&dronecan_thread, dronecan_task, DRONECAN_TASK_STACKSIZE,
                     "dronecan_task");
    }

    /* All tasks run forever; block here for the lifetime of the program. */

    pthread_join(gps_thread, NULL);
    pthread_join(imu_thread, NULL);
    pthread_join(mavlink_thread, NULL);
    if (have_dronecan) {
        pthread_join(dronecan_thread, NULL);
    }

    return 0;
}
