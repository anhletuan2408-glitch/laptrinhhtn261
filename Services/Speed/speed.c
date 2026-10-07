/**
 * @file    speed.c
 * @brief   Vehicle speed / distance estimation from Hall wheel pulses.
 */
#include "speed.h"

#include <stddef.h>

/**
 * @brief Validate the configuration and initialise the estimator.
 */
int speed_init(speed_t *s, const speed_config_t *cfg)
{
    static const speed_config_t def = SPEED_CONFIG_DEFAULT;
    speed_config_t c;

    if (s == NULL) {
        return -1;
    }
    c = (cfg != NULL) ? *cfg : def;

    /* Written so that NaN fails every comparison. */
    if (!(c.wheel_circumference_m > 0.0f) || c.pulses_per_rev == 0u ||
        !(c.ema_alpha > 0.0f && c.ema_alpha <= 1.0f)) {
        return -1;
    }

    s->cfg              = c;
    s->dist_per_pulse_m = c.wheel_circumference_m / (float)c.pulses_per_rev;
    s->v_mps            = 0.0f;
    s->last_update_us   = 0u;
    s->pulse_base       = 0u;
    s->has_update       = 0u;
    return 0;
}

/**
 * @brief Compute a new speed/acceleration/distance estimate from a Hall snapshot.
 */
void speed_update(speed_t *s, const hall_snapshot_t *snap, uint64_t now_us, speed_result_t *out)
{
    double   dpp;
    double   raw = 0.0;
    float    v_prev;
    float    accel = 0.0f;
    uint64_t since;

    if (s == NULL || snap == NULL) {
        return;
    }
    dpp    = (double)s->dist_per_pulse_m;
    v_prev = s->v_mps;

    /* Time since the last accepted pulse; clamp to 0 if the clock appears to go backwards. */
    since = (snap->pulse_count != 0u && now_us > snap->last_pulse_us)
                ? (now_us - snap->last_pulse_us) : 0u;

    if (snap->pulse_count == 0u || since >= (uint64_t)s->cfg.stop_timeout_us) {
        raw = 0.0;                                           /* step 1 */
    } else if (snap->last_period_us == 0u) {
        raw = 0.0;                                           /* step 2 */
    } else {
        raw = dpp / ((double)snap->last_period_us * 1e-6);   /* step 3 */
        if (since > (uint64_t)snap->last_period_us) {
            double decayed = dpp / ((double)since * 1e-6);
            if (decayed < raw) {
                raw = decayed;
            }
        }
    }

    /* Step 4: EMA filter, immediate stop on zero. */
    if (raw == 0.0) {
        s->v_mps = 0.0f;
    } else {
        s->v_mps = s->v_mps + s->cfg.ema_alpha * ((float)raw - s->v_mps);
    }

    /* Step 5: acceleration from the speed derivative. */
    if (s->has_update && now_us > s->last_update_us) {
        double dt = (double)(now_us - s->last_update_us) * 1e-6;
        accel = (float)((double)(s->v_mps - v_prev) / dt);
    }
    s->last_update_us = now_us;
    s->has_update     = 1u;

    /* Distance is relative to pulse_base; re-base if the counter went backwards (hall_reset). */
    if (snap->pulse_count < s->pulse_base) {
        s->pulse_base = snap->pulse_count;
    }

    if (out != NULL) {
        out->speed_mps   = s->v_mps;
        out->speed_kmh   = s->v_mps * 3.6f;
        out->accel_mps2  = accel;
        out->distance_m  = (float)((double)(snap->pulse_count - s->pulse_base) * dpp);
        out->pulse_count = snap->pulse_count;
    }
}

/**
 * @brief Zero the trip distance at the given current pulse count.
 */
void speed_reset_distance(speed_t *s, uint32_t current_pulse_count)
{
    if (s == NULL) {
        return;
    }
    s->pulse_base = current_pulse_count;
}
