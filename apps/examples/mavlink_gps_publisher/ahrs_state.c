#include "ahrs_state.h"
#include <pthread.h>

static ahrs_state_t s_state;
static pthread_mutex_t s_mutex;

void AhrsState_Init(void)
{
    pthread_mutex_init(&s_mutex, NULL);
    s_state = (ahrs_state_t){0};
}

void AhrsState_Update(float roll_rad, float pitch_rad, float yaw_rad,
                       float rollspeed_rads, float pitchspeed_rads, float yawspeed_rads)
{
    pthread_mutex_lock(&s_mutex);
    s_state.valid = true;
    s_state.roll_rad = roll_rad;
    s_state.pitch_rad = pitch_rad;
    s_state.yaw_rad = yaw_rad;
    s_state.rollspeed_rads = rollspeed_rads;
    s_state.pitchspeed_rads = pitchspeed_rads;
    s_state.yawspeed_rads = yawspeed_rads;
    pthread_mutex_unlock(&s_mutex);
}

void AhrsState_GetSnapshot(ahrs_state_t *out)
{
    pthread_mutex_lock(&s_mutex);
    *out = s_state;
    pthread_mutex_unlock(&s_mutex);
}
