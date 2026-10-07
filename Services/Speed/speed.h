/**
 * @file    speed.h
 * @brief   Vehicle speed / distance estimation from Hall wheel pulses.
 *
 * Pure computation, no hardware access: feed it hall_snapshot_t + current time.
 *
 * Algorithm (speed_update):
 *   dist_per_pulse = wheel_circumference_m / pulses_per_rev
 *   1. No pulse yet, or (now - last_pulse) >= stop_timeout_us  -> raw = 0.
 *   2. Fewer than 2 pulses (last_period_us == 0)                -> raw = 0.
 *   3. raw = dist_per_pulse / (last_period_us * 1e-6)   [m/s]
 *      Decay while decelerating: if (now - last_pulse) > last_period_us,
 *      the wheel is slower than the last period says, so
 *      raw = min(raw, dist_per_pulse / ((now - last_pulse) * 1e-6)).
 *   4. Filter: v = v + ema_alpha * (raw - v); if raw == 0 then v = 0 immediately.
 *   5. accel_mps2 = (v - v_prev) / dt, dt = now - prev_update (0 on first call / dt == 0).
 *   distance_m = pulse_count * dist_per_pulse (relative to pulse_count at init/reset).
 */
#ifndef SPEED_H
#define SPEED_H

#include <stdint.h>
#include "hall.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float    wheel_circumference_m;  /* e.g. 0.21 m for a 67 mm RC wheel, 1.9 m for a car */
    uint8_t  pulses_per_rev;         /* magnets per wheel, >= 1                          */
    uint32_t stop_timeout_us;        /* no pulse for this long -> speed 0                 */
    float    ema_alpha;              /* 0 < alpha <= 1, 1 = no filtering                  */
} speed_config_t;

#define SPEED_CONFIG_DEFAULT { 0.21f, 1u, 2000000u, 0.5f }

typedef struct {
    float    speed_mps;
    float    speed_kmh;
    float    accel_mps2;     /* longitudinal accel from speed derivative (+ = speeding up) */
    float    distance_m;
    uint32_t pulse_count;
} speed_result_t;

typedef struct {
    speed_config_t cfg;
    float          dist_per_pulse_m;
    float          v_mps;
    uint64_t       last_update_us;
    uint32_t       pulse_base;      /* pulse_count at reset */
    uint8_t        has_update;
} speed_t;

/** Returns 0 on success, -1 on invalid cfg (NULL obj, circumference <= 0,
 *  pulses_per_rev == 0, alpha not in (0,1]). cfg NULL -> SPEED_CONFIG_DEFAULT. */
int speed_init(speed_t *s, const speed_config_t *cfg);

/** Compute a new estimate. Call periodically (e.g. 10-50 Hz) from a task. */
void speed_update(speed_t *s, const hall_snapshot_t *snap, uint64_t now_us, speed_result_t *out);

/** Reset trip distance to 0 at the given current pulse_count. */
void speed_reset_distance(speed_t *s, uint32_t current_pulse_count);

#ifdef __cplusplus
}
#endif

#endif /* SPEED_H */
