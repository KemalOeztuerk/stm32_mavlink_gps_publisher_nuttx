#include "baro_state.h"
#include "clock_ms.h"
#include <pthread.h>

static baro_state_t s_state;
static pthread_mutex_t s_mutex;
static uint32_t s_pressure_last_tick;
static uint32_t s_temperature_last_tick;

void BaroState_Init(void)
{
    pthread_mutex_init(&s_mutex, NULL);
    s_state = (baro_state_t){0};
    s_pressure_last_tick = clock_now_ms();
    s_temperature_last_tick = s_pressure_last_tick;
}

void BaroState_UpdatePressure(float pressure_pa)
{
    pthread_mutex_lock(&s_mutex);
    s_state.pressure_valid = true;
    s_state.pressure_pa = pressure_pa;
    s_pressure_last_tick = clock_now_ms();
    pthread_mutex_unlock(&s_mutex);
}

void BaroState_UpdateTemperature(float temperature_degc)
{
    pthread_mutex_lock(&s_mutex);
    s_state.temperature_valid = true;
    s_state.temperature_degc = temperature_degc;
    s_temperature_last_tick = clock_now_ms();
    pthread_mutex_unlock(&s_mutex);
}

void BaroState_GetSnapshot(baro_state_t *out)
{
    pthread_mutex_lock(&s_mutex);
    *out = s_state;
    uint32_t now = clock_now_ms();
    out->pressure_age_ms = now - s_pressure_last_tick;
    out->temperature_age_ms = now - s_temperature_last_tick;
    pthread_mutex_unlock(&s_mutex);
}
