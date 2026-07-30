/* Shared, mutex-protected GPS + heading state written by dronecan_task()
 * (Here4 Fix2/Auxiliary/compass decoding) and read by mavlink_tx_task(). */
#ifndef NAV_STATE_H
#define NAV_STATE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool gps_fix_valid;          /* Fix2 status >= 2D fix */
    uint8_t fix_type;            /* MAVLink GPS_FIX_TYPE: 0/1 none, 2=2D, 3=3D,
                                  * 4=DGPS, 5=RTK float, 6=RTK fixed */
    int32_t lat_degE7;
    int32_t lon_degE7;
    float alt_m;                 /* MSL altitude, meters */
    float speed_mps;             /* horizontal ground speed */
    float course_deg;            /* true course over ground, 0-360 */
    float vel_d_mps;             /* NED down velocity (negative = climbing) */
    uint8_t sats_used;           /* from Fix2 */
    float pdop;                  /* from Fix2 (its only precision figure) */
    uint32_t gps_age_ms;         /* time since last GPS update, filled by GetSnapshot */

    bool dop_valid;              /* gnss.Auxiliary seen at least once */
    float hdop;                  /* from Auxiliary */
    float vdop;                  /* from Auxiliary */
    uint8_t sats_visible;        /* from Auxiliary */
    uint32_t dop_age_ms;         /* time since last Auxiliary update */

    bool heading_valid;
    float heading_deg;           /* compass heading, 0-360, 0=unavailable marker handled by caller */
    uint32_t heading_age_ms;     /* time since last heading update, filled by GetSnapshot */
} nav_state_t;

void NavState_Init(void);

void NavState_UpdateGps(bool fix_valid, uint8_t fix_type, int32_t lat_degE7, int32_t lon_degE7,
                         float alt_m, float speed_mps, float course_deg, float vel_d_mps,
                         uint8_t sats_used, float pdop);

/* DOP/satellite-count detail from uavcan.equipment.gnss.Auxiliary. */
void NavState_UpdateDops(float hdop, float vdop, uint8_t sats_used, uint8_t sats_visible);

void NavState_UpdateHeading(float heading_deg);

/* Copies out the current state; *_age_ms fields are computed relative to now. */
void NavState_GetSnapshot(nav_state_t *out);

#ifdef __cplusplus
}
#endif

#endif /* NAV_STATE_H */
