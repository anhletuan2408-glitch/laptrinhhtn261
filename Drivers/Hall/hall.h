/**
 * @file    hall.h
 * @brief   Portable Hall-effect wheel sensor (pulse capture) driver.
 *
 * The GPIO interrupt of the platform calls hall_on_pulse() with a
 * microsecond timestamp. Tasks read a consistent copy with hall_get_snapshot().
 *
 * ISR safety: the 64-bit timestamp cannot be accessed atomically on a 32-bit
 * MCU and ESP32 is dual-core, so both hall_on_pulse() and hall_get_snapshot()
 * run inside HALL_ENTER_CRITICAL()/HALL_EXIT_CRITICAL(). In hall.c these map to
 * portENTER_CRITICAL_SAFE()/portEXIT_CRITICAL_SAFE() on a static portMUX when
 * ESP_PLATFORM is defined, and to no-ops on host builds.
 */
#ifndef HALL_H
#define HALL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t min_interval_us;   /* pulses closer than this are rejected as bounce/noise */
} hall_config_t;

/** 2 ms debounce -> max 500 pulses/s, far above any realistic wheel rate. */
#define HALL_CONFIG_DEFAULT { 2000u }

typedef struct {
    uint32_t pulse_count;      /* accepted pulses since init / reset           */
    uint32_t last_period_us;   /* interval between the last two accepted pulses, 0 if < 2 pulses */
    uint64_t last_pulse_us;    /* timestamp of last accepted pulse, 0 if none  */
    uint32_t rejected_count;   /* pulses dropped by debounce (diagnostics)     */
} hall_snapshot_t;

typedef struct {
    hall_config_t            cfg;
    volatile uint32_t        pulse_count;
    volatile uint32_t        last_period_us;
    volatile uint64_t        last_pulse_us;
    volatile uint32_t        rejected_count;
    volatile bool            has_pulse;
} hall_t;

/** @param cfg NULL -> HALL_CONFIG_DEFAULT. Clears all counters. */
void hall_init(hall_t *h, const hall_config_t *cfg);

/**
 * Called from the GPIO ISR on the active edge. Must be short and non-blocking.
 * First pulse: stores timestamp only (period stays 0).
 * Pulse with (now - last) < min_interval_us: rejected_count++, nothing else changes.
 * Otherwise: last_period_us = now - last (saturate at UINT32_MAX), count++, last = now.
 */
void hall_on_pulse(hall_t *h, uint64_t timestamp_us);

/** Copy the current state atomically w.r.t. hall_on_pulse(). */
void hall_get_snapshot(const hall_t *h, hall_snapshot_t *out);

/** Reset counters (e.g. trip reset). Keeps cfg. */
void hall_reset(hall_t *h);

#ifdef __cplusplus
}
#endif

#endif /* HALL_H */
