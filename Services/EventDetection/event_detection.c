/**
 * @file    event_detection.c
 * @brief   Hard-brake and crash detection (see event_detection.h for the spec).
 */
#include "event_detection.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/** @brief Wrap-safe "at least `span` ms elapsed between `since` and `now`". */
static bool elapsed_ge(uint32_t now, uint32_t since, uint32_t span)
{
    return (uint32_t)(now - since) >= span;
}

/** @brief Fill an output event (no-op when evt is NULL). */
static void fill_event(event_t *evt, event_type_t type, uint32_t ts,
                       float peak_g, float speed_kmh, float threshold_g)
{
    if (evt == NULL) {
        return;
    }
    evt->type         = type;
    evt->timestamp_ms = ts;
    evt->peak_g       = peak_g;
    evt->speed_kmh    = speed_kmh;
    evt->severity     = peak_g / threshold_g;
}

/** @brief Validate a configuration (NaN-safe: comparisons are written to fail on NaN). */
static bool cfg_valid(const event_detection_config_t *c)
{
    return (c->brake_threshold_g > 0.0f) &&
           (c->crash_threshold_g > 0.0f) &&
           (c->brake_min_speed_kmh >= 0.0f) &&
           (c->crash_min_samples > 0u) &&
           (c->lpf_alpha > 0.0f) && (c->lpf_alpha <= 1.0f);
}

int event_detection_init(event_detector_t *d, const event_detection_config_t *cfg)
{
    static const event_detection_config_t k_default = EVENT_DETECTION_CONFIG_DEFAULT;

    if (d == NULL) {
        return -1;
    }
    const event_detection_config_t *use = (cfg != NULL) ? cfg : &k_default;
    if (!cfg_valid(use)) {
        return -1;
    }

    memset(d, 0, sizeof(*d));
    d->cfg = *use;
    return 0;
}

/** @brief Crash detector step; returns true when a crash fires. */
static bool crash_step(event_detector_t *d, const event_sample_t *s, event_t *evt)
{
    const float mag = sqrtf(s->ax_g * s->ax_g + s->ay_g * s->ay_g + s->az_g * s->az_g);

    /* Inside the cooldown window samples are ignored entirely. */
    if (d->crash_has_last &&
        !elapsed_ge(s->timestamp_ms, d->crash_last_event_ms, d->cfg.crash_cooldown_ms)) {
        d->crash_count  = 0;
        d->crash_peak_g = 0.0f;
        return false;
    }

    if (mag < d->cfg.crash_threshold_g) {
        d->crash_count  = 0;
        d->crash_peak_g = 0.0f;
        return false;
    }

    if (d->crash_count == 0u || mag > d->crash_peak_g) {
        d->crash_peak_g = mag;
    }
    if (d->crash_count < UINT8_MAX) {
        d->crash_count++;
    }
    if (d->crash_count < d->cfg.crash_min_samples) {
        return false;
    }

    fill_event(evt, EVT_CRASH, s->timestamp_ms, d->crash_peak_g, s->speed_kmh,
               d->cfg.crash_threshold_g);
    d->crash_has_last     = true;
    d->crash_last_event_ms = s->timestamp_ms;
    d->crash_count        = 0;
    d->crash_peak_g       = 0.0f;
    return true;
}

/** @brief Hard-brake state machine step; returns true when a brake event fires. */
static bool brake_step(event_detector_t *d, const event_sample_t *s, event_t *evt)
{
    const bool below = (d->ax_f <= -d->cfg.brake_threshold_g);

    if (d->brake_fired) {
        /* Waiting for ax_f to recover above the threshold. */
        if (!below) {
            d->brake_fired = false;
        }
        return false;
    }

    if (!d->brake_armed) {
        if (!below || s->speed_kmh < d->cfg.brake_min_speed_kmh) {
            return false;
        }
        if (d->brake_has_last &&
            !elapsed_ge(s->timestamp_ms, d->brake_last_event_ms, d->cfg.brake_cooldown_ms)) {
            return false;
        }
        d->brake_armed       = true;
        d->brake_start_ms    = s->timestamp_ms;
        d->brake_peak_g      = -d->ax_f;
        d->brake_start_speed = s->speed_kmh;
    } else {
        if (!below) {
            d->brake_armed = false;   /* released too early: no event */
            return false;
        }
        if (-d->ax_f > d->brake_peak_g) {
            d->brake_peak_g = -d->ax_f;
        }
    }

    if (!elapsed_ge(s->timestamp_ms, d->brake_start_ms, d->cfg.brake_min_duration_ms)) {
        return false;
    }

    fill_event(evt, EVT_HARD_BRAKE, s->timestamp_ms, d->brake_peak_g,
               d->brake_start_speed, d->cfg.brake_threshold_g);
    d->brake_armed         = false;
    d->brake_fired         = true;
    d->brake_has_last      = true;
    d->brake_last_event_ms = s->timestamp_ms;
    return true;
}

bool event_detection_process(event_detector_t *d, const event_sample_t *s, event_t *evt)
{
    if (evt != NULL) {
        evt->type = EVT_NONE;
    }
    if (d == NULL || s == NULL) {
        return false;
    }

    /* Low-pass filter, initialised with the first sample. */
    if (!d->lpf_init) {
        d->ax_f     = s->ax_g;
        d->lpf_init = true;
    } else {
        d->ax_f += d->cfg.lpf_alpha * (s->ax_g - d->ax_f);
    }

    /* Crash has priority over brake. */
    if (crash_step(d, s, evt)) {
        /* A crash always cancels any armed brake and holds the brake detector
         * off until ax_f has recovered above -threshold. Even when no brake was
         * armed at the impact sample, the impact drags ax_f strongly negative
         * afterwards, which would otherwise report a redundant hard-brake
         * shortly after the CRASH event. */
        d->brake_armed = false;
        d->brake_fired = true;
        return true;
    }

    return brake_step(d, s, evt);
}

const char *event_type_str(event_type_t t)
{
    switch (t) {
    case EVT_NONE:       return "NONE";
    case EVT_HARD_BRAKE: return "HARD_BRAKE";
    case EVT_CRASH:      return "CRASH";
    default:             return "UNKNOWN";
    }
}
