#include "nav_state.h"
#include "clock_ms.h"
#include <pthread.h>

static nav_state_t s_state;
static pthread_mutex_t s_mutex;
static uint32_t s_gps_last_tick;
static uint32_t s_dop_last_tick;
static uint32_t s_heading_last_tick;

void NavState_Init(void)
{
    pthread_mutex_init(&s_mutex, NULL);
    s_state = (nav_state_t){0};
    s_gps_last_tick = clock_now_ms();
    s_dop_last_tick = s_gps_last_tick;
    s_heading_last_tick = s_gps_last_tick;
}

void NavState_UpdateGps(bool fix_valid, uint8_t fix_type, int32_t lat_degE7, int32_t lon_degE7,
                         float alt_m, float speed_mps, float course_deg, float vel_d_mps,
                         uint8_t sats_used, float pdop)
{
    pthread_mutex_lock(&s_mutex);
    s_state.gps_fix_valid = fix_valid;
    s_state.fix_type = fix_type;
    s_state.lat_degE7 = lat_degE7;
    s_state.lon_degE7 = lon_degE7;
    s_state.alt_m = alt_m;
    s_state.speed_mps = speed_mps;
    s_state.course_deg = course_deg;
    s_state.vel_d_mps = vel_d_mps;
    s_state.sats_used = sats_used;
    s_state.pdop = pdop;
    s_gps_last_tick = clock_now_ms();
    pthread_mutex_unlock(&s_mutex);
}

void NavState_UpdateDops(float hdop, float vdop, uint8_t sats_used, uint8_t sats_visible)
{
    pthread_mutex_lock(&s_mutex);
    s_state.dop_valid = true;
    s_state.hdop = hdop;
    s_state.vdop = vdop;
    s_state.sats_used = sats_used;
    s_state.sats_visible = sats_visible;
    s_dop_last_tick = clock_now_ms();
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
    out->dop_age_ms = now - s_dop_last_tick;
    out->heading_age_ms = now - s_heading_last_tick;
    pthread_mutex_unlock(&s_mutex);
}
