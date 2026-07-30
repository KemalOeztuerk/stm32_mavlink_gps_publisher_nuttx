/****************************************************************************
 * apps/examples/mavlink_gps_publisher/mavlink_gps_publisher_main.c
 *
 * Wires together the Here4/DroneCAN sensor listener (GNSS, compass, IMU,
 * barometer, plus the dynamic-node-ID allocation server) and the MAVLink
 * telemetry publisher, each running as its own pthread.
 ****************************************************************************/

#include <nuttx/config.h>

#include <pthread.h>
#include <syslog.h>

#include "nav_state.h"
#include "imu_state.h"
#include "ahrs_state.h"
#include "ahrs_filter.h"
#include "baro_state.h"
#include "mavlink_link.h"
#include "dronecan_gnss.h"

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
    BaroState_Init();

    if (MavlinkLink_Init(CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER_MAVLINK_DEVPATH) < 0) {
        syslog(LOG_ERR, "mavlink_gps_publisher: failed to open MAVLink device %s\n",
               CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER_MAVLINK_DEVPATH);
        return -1;
    }

    /* The Here4 on CAN1 is the only sensor source, so failure to open the
     * CAN device is fatal. */
    if (DroneCanGnss_Init(CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER_CAN_DEVPATH) < 0) {
        syslog(LOG_ERR, "mavlink_gps_publisher: failed to open CAN device %s\n",
               CONFIG_EXAMPLES_MAVLINK_GPS_PUBLISHER_CAN_DEVPATH);
        return -1;
    }

    pthread_t mavlink_thread;
    pthread_t dronecan_thread;

    start_thread(&dronecan_thread, dronecan_task, DRONECAN_TASK_STACKSIZE,
                 "dronecan_task");
    start_thread(&mavlink_thread, mavlink_tx_task, MAVLINK_TX_STACKSIZE,
                 "mavlink_tx_task");

    /* All tasks run forever; block here for the lifetime of the program. */

    pthread_join(dronecan_thread, NULL);
    pthread_join(mavlink_thread, NULL);

    return 0;
}
