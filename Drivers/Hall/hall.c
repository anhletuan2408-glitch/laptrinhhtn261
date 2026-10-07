/**
 * @file    hall.c
 * @brief   Portable Hall-effect wheel sensor (pulse capture) driver.
 */
#include "hall.h"

#include <stddef.h>

#ifdef ESP_PLATFORM
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
static portMUX_TYPE s_hall_mux = portMUX_INITIALIZER_UNLOCKED;
#define HALL_ENTER_CRITICAL() portENTER_CRITICAL_SAFE(&s_hall_mux)
#define HALL_EXIT_CRITICAL()  portEXIT_CRITICAL_SAFE(&s_hall_mux)
#define HALL_ISR_ATTR         IRAM_ATTR
#else
#define HALL_ENTER_CRITICAL() ((void)0)
#define HALL_EXIT_CRITICAL()  ((void)0)
#define HALL_ISR_ATTR
#endif

/**
 * @brief Clear all counters and timestamps (caller holds the critical section if needed).
 */
static void hall_clear(hall_t *h)
{
    h->pulse_count    = 0u;
    h->last_period_us = 0u;
    h->last_pulse_us  = 0u;
    h->rejected_count = 0u;
    h->has_pulse      = false;
}

/**
 * @brief Initialise the driver state; cfg NULL selects HALL_CONFIG_DEFAULT.
 */
void hall_init(hall_t *h, const hall_config_t *cfg)
{
    static const hall_config_t def = HALL_CONFIG_DEFAULT;

    if (h == NULL) {
        return;
    }
    h->cfg = (cfg != NULL) ? *cfg : def;
    HALL_ENTER_CRITICAL();
    hall_clear(h);
    HALL_EXIT_CRITICAL();
}

/**
 * @brief Register one pulse edge (ISR context). Applies debounce and updates the period.
 */
void HALL_ISR_ATTR hall_on_pulse(hall_t *h, uint64_t timestamp_us)
{
    if (h == NULL) {
        return;
    }

    HALL_ENTER_CRITICAL();
    if (!h->has_pulse) {
        /* First pulse: timestamp only, period stays 0. */
        h->last_pulse_us  = timestamp_us;
        h->last_period_us = 0u;
        h->pulse_count    = 1u;
        h->has_pulse      = true;
    } else if (timestamp_us < h->last_pulse_us) {
        /* Timestamp went backwards: reject. */
        h->rejected_count++;
    } else {
        uint64_t delta = timestamp_us - h->last_pulse_us;
        if (delta < (uint64_t)h->cfg.min_interval_us) {
            h->rejected_count++;
        } else {
            h->last_period_us = (delta > (uint64_t)UINT32_MAX) ? UINT32_MAX : (uint32_t)delta;
            h->pulse_count++;
            h->last_pulse_us = timestamp_us;
        }
    }
    HALL_EXIT_CRITICAL();
}

/**
 * @brief Copy the current state consistently with respect to hall_on_pulse().
 */
void hall_get_snapshot(const hall_t *h, hall_snapshot_t *out)
{
    if (out == NULL) {
        return;
    }
    if (h == NULL) {
        out->pulse_count    = 0u;
        out->last_period_us = 0u;
        out->last_pulse_us  = 0u;
        out->rejected_count = 0u;
        return;
    }

    HALL_ENTER_CRITICAL();
    out->pulse_count    = h->pulse_count;
    out->last_period_us = h->last_period_us;
    out->last_pulse_us  = h->last_pulse_us;
    out->rejected_count = h->rejected_count;
    HALL_EXIT_CRITICAL();
}

/**
 * @brief Reset all counters (e.g. trip reset); configuration is kept.
 */
void hall_reset(hall_t *h)
{
    if (h == NULL) {
        return;
    }
    HALL_ENTER_CRITICAL();
    hall_clear(h);
    HALL_EXIT_CRITICAL();
}
