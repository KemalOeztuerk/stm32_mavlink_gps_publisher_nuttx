/* Here4 (DroneCAN) sensor listener + dynamic-node-ID allocation server
 * over CAN1.
 *
 * Decodes every sensor broadcast a Here4 emits:
 *   - uavcan.equipment.gnss.Fix2           -> nav_state (position/velocity/fix)
 *   - uavcan.equipment.gnss.Auxiliary      -> nav_state (HDOP/VDOP/satellites)
 *   - uavcan.equipment.ahrs.MagneticFieldStrength2
 *                                          -> tilt-compensated heading
 *   - uavcan.equipment.ahrs.RawIMU         -> AHRS complementary filter
 *   - uavcan.equipment.air_data.StaticPressure / StaticTemperature
 *                                          -> baro_state
 *
 * This node also acts as the bus's dynamic node allocation (DNA) server:
 * a factory-fresh Here4 boots with no node ID and requests one anonymously
 * (uavcan.protocol.dynamic_node_id.Allocation); we run the centralized
 * allocator side of that handshake, so no static node ID ever needs to be
 * configured on the Here4. Our own NodeStatus is broadcast at 1Hz so the
 * Here4 sees a live bus master.
 */
#ifndef DRONECAN_GNSS_H
#define DRONECAN_GNSS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Live bus/allocation counters for the STATUSTEXT diagnostic (the board has
 * no debug console, so the MAVLink link doubles as one). */
typedef struct {
    uint32_t rx_frames;       /* raw CAN frames received */
    uint32_t tx_errors;       /* frames dropped on CAN write error */
    uint32_t alloc_requests;  /* DNA request stages received (anonymous) */
    uint8_t  allocated_id;    /* last node ID handed out, 0 = none yet */
    uint8_t  remote_node_id;  /* last non-anonymous source heard, 0 = none */
    uint32_t fix2_count;      /* GNSS Fix2 messages decoded */
    uint32_t mag_count;       /* magnetometer messages decoded (any variant) */
    uint8_t  remote_health;   /* peripheral's NodeStatus health: 0=OK 1=warn
                               * 2=error 3=critical */
    int16_t  raw_acc[3];      /* last RawIMU accel, untransformed, 0.1 m/s^2 */
    int16_t  raw_mag[3];      /* last mag field, untransformed, 0.01 Gauss */
    int16_t  raw_gyro[3];     /* rate_gyro_latest, untransformed, 0.01 rad/s */
    int16_t  raw_gyro_int[3]; /* rate_gyro_integral/interval, 0.01 rad/s */
} dronecan_stats_t;

void DroneCanGnss_GetStats(dronecan_stats_t *out);

/* Copies out (and clears) the latest Here4 debug LogMessage, formatted for
 * a STATUSTEXT. Returns false if none arrived since the last call. */
bool DroneCanGnss_TakeLogMessage(char *out, size_t out_len);

/* Opens the CAN device and initializes libcanard. Call once from main
 * before spawning dronecan_task(). Returns 0 on success, -1 on failure. */
int DroneCanGnss_Init(const char *devpath);

/* pthread entry: receives and decodes CAN frames, serves node ID
 * allocation requests, and broadcasts NodeStatus, forever. */
void *dronecan_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* DRONECAN_GNSS_H */
