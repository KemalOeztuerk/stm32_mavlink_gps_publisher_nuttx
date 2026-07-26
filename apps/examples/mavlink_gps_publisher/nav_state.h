/* Shared, mutex-protected GPS + heading state written by gps_task()
 * and read by mavlink_tx_task(). */
#ifndef NAV_STATE_H
#define NAV_STATE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* NAV_GPS_SOURCE_CAN (Here4/DroneCAN) always wins over NAV_GPS_SOURCE_NMEA
 * while it's actively updating; NMEA is only accepted as a fallback once
 * no CAN fix has arrived for a couple of seconds. See NavState_UpdateGps(). */
typedef enum {
    NAV_GPS_SOURCE_NMEA,
    NAV_GPS_SOURCE_CAN,
} nav_gps_source_t;

typedef struct {
    bool gps_fix_valid;          /* GPRMC status == 'A' */
    uint8_t fix_type;            /* 0=no fix, 2=2D, 3=3D (from GGA fix quality) */
    int32_t lat_degE7;
    int32_t lon_degE7;
    float alt_m;                 /* MSL altitude, meters */
    float speed_mps;             /* ground speed */
    float course_deg;            /* true course over ground, 0-360 */
    uint8_t satellites_visible;
    float hdop;
    uint32_t gps_age_ms;         /* time since last GPS update, filled by GetSnapshot */

    bool heading_valid;
    float heading_deg;           /* compass heading, 0-360, 0=unavailable marker handled by caller */
    uint32_t heading_age_ms;     /* time since last heading update, filled by GetSnapshot */
} nav_state_t;

void NavState_Init(void);

void NavState_UpdateGps(bool fix_valid, uint8_t fix_type, int32_t lat_degE7, int32_t lon_degE7,
                         float alt_m, float speed_mps, float course_deg,
                         uint8_t satellites_visible, float hdop,
                         nav_gps_source_t source);

void NavState_UpdateHeading(float heading_deg);

/* Copies out the current state; *_age_ms fields are computed relative to now. */
void NavState_GetSnapshot(nav_state_t *out);

#ifdef __cplusplus
}
#endif

#endif /* NAV_STATE_H */
