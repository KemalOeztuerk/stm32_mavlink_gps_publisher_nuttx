/* Shared, mutex-protected barometer state written by dronecan_task()
 * (Here4 StaticPressure/StaticTemperature broadcasts) and read by
 * mavlink_tx_task(). Pressure and temperature arrive as two independent
 * DroneCAN messages, so each has its own valid flag and age. */
#ifndef BARO_STATE_H
#define BARO_STATE_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool pressure_valid;
    float pressure_pa;           /* static (absolute) pressure, Pascal */
    uint32_t pressure_age_ms;    /* filled by GetSnapshot */

    bool temperature_valid;
    float temperature_degc;      /* static air temperature, Celsius */
    uint32_t temperature_age_ms; /* filled by GetSnapshot */
} baro_state_t;

void BaroState_Init(void);

void BaroState_UpdatePressure(float pressure_pa);

void BaroState_UpdateTemperature(float temperature_degc);

/* Copies out the current state; *_age_ms fields are computed relative to now. */
void BaroState_GetSnapshot(baro_state_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BARO_STATE_H */
