#include "nav_state.h"
#include "clock_ms.h"
#include <pthread.h>

/* CAN (Here4) is treated as "alive" -- and takes priority over NMEA -- for
 * this long after its last update. */
#define NAV_CAN_GPS_PRIORITY_WINDOW_MS 2000U

static nav_state_t s_state;
static pthread_mutex_t s_mutex;
static uint32_t s_gps_last_tick;
static uint32_t s_heading_last_tick;
static uint32_t s_can_gps_last_tick;

void NavState_Init(void)
{
    pthread_mutex_init(&s_mutex, NULL);
    s_state = (nav_state_t){0};
    s_gps_last_tick = clock_now_ms();
    s_heading_last_tick = s_gps_last_tick;
    s_can_gps_last_tick = 0U;
}

void NavState_UpdateGps(bool fix_valid, uint8_t fix_type, int32_t lat_degE7, int32_t lon_degE7,
                         float alt_m, float speed_mps, float course_deg,
                         uint8_t satellites_visible, float hdop,
                         nav_gps_source_t source)
{
    pthread_mutex_lock(&s_mutex);

    uint32_t now = clock_now_ms();
    if (source == NAV_GPS_SOURCE_CAN) {
        s_can_gps_last_tick = now;
    } else if ((now - s_can_gps_last_tick) < NAV_CAN_GPS_PRIORITY_WINDOW_MS) {
        /* A CAN (Here4) fix is still active: ignore the NMEA update. */
        pthread_mutex_unlock(&s_mutex);
        return;
    }

    s_state.gps_fix_valid = fix_valid;
    s_state.fix_type = fix_type;
    s_state.lat_degE7 = lat_degE7;
    s_state.lon_degE7 = lon_degE7;
    s_state.alt_m = alt_m;
    s_state.speed_mps = speed_mps;
    s_state.course_deg = course_deg;
    s_state.satellites_visible = satellites_visible;
    s_state.hdop = hdop;
    s_gps_last_tick = now;
    pthread_mutex_unlock(&s_mutex);
}

void NavState_UpdateHeading(float heading_deg)
{
    pthread_mutex_lock(&s_mutex);
    s_state.heading_valid = true;
    s_state.heading_deg = heading_deg;
    s_heading_last_tick = clock_now_ms();
    pthread_mutex_unlock(&s_mutex);
}

void NavState_GetSnapshot(nav_state_t *out)
{
    pthread_mutex_lock(&s_mutex);
    *out = s_state;
    uint32_t now = clock_now_ms();
    out->gps_age_ms = now - s_gps_last_tick;
    out->heading_age_ms = now - s_heading_last_tick;
    pthread_mutex_unlock(&s_mutex);
}
