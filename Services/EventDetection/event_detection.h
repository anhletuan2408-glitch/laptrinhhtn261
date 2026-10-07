/**
 * @file    event_detection.h
 * @brief   Hard-brake and crash detection from IMU + speed.
 *
 * Pure computation. The sensor task feeds one sample per IMU read (~100 Hz)
 * and forwards any returned event to the logger / HMI.
 *
 * HARD BRAKE (state machine, uses low-pass filtered forward accel):
 *   ax_f = ax_f + lpf_alpha * (ax_g - ax_f)
 *   Arm when ax_f <= -brake_threshold_g AND speed_kmh >= brake_min_speed_kmh.
 *   While armed, track peak (most negative ax_f, reported as positive g) and
 *   the speed at arm time. Disarm (no event) as soon as ax_f > -brake_threshold_g
 *   before brake_min_duration_ms elapsed.
 *   Fire once when armed for >= brake_min_duration_ms (event timestamp = that sample).
 *   After firing, no new brake event until ax_f recovers above -brake_threshold_g
 *   AND brake_cooldown_ms has passed since the event.
 *
 * CRASH (uses raw, unfiltered accel - impacts are short):
 *   mag = sqrt(ax^2 + ay^2 + az^2) [g]  (1 g at rest)
 *   Fire when mag >= crash_threshold_g on crash_min_samples consecutive samples.
 *   peak_g = max mag of those samples; severity = peak_g / crash_threshold_g.
 *   No new crash event for crash_cooldown_ms after one.
 *   A crash cancels any armed brake (crash has priority; at most 1 event per call).
 *
 * Timestamps are uint32_t ms; use (int32_t)(a - b) / unsigned subtraction so
 * wrap-around after ~49 days is handled.
 */
#ifndef EVENT_DETECTION_H
#define EVENT_DETECTION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    EVT_NONE = 0,
    EVT_HARD_BRAKE,
    EVT_CRASH
} event_type_t;

typedef struct {
    float    brake_threshold_g;      /* 0.45 g  ~ 4.4 m/s^2 */
    uint32_t brake_min_duration_ms;  /* 150 ms              */
    float    brake_min_speed_kmh;    /* 5 km/h              */
    uint32_t brake_cooldown_ms;      /* 2000 ms             */
    float    crash_threshold_g;      /* 3.0 g               */
    uint8_t  crash_min_samples;      /* 2 (= 20 ms @100 Hz) */
    uint32_t crash_cooldown_ms;      /* 5000 ms             */
    float    lpf_alpha;              /* 0.3                 */
} event_detection_config_t;

#define EVENT_DETECTION_CONFIG_DEFAULT { 0.45f, 150u, 5.0f, 2000u, 3.0f, 2u, 5000u, 0.3f }

/** One input sample. Axis convention as in mpu6050.h (+X forward). */
typedef struct {
    uint32_t timestamp_ms;
    float    ax_g, ay_g, az_g;
    float    gx_dps, gy_dps, gz_dps;
    float    speed_kmh;
} event_sample_t;

typedef struct {
    event_type_t type;
    uint32_t     timestamp_ms;
    float        peak_g;      /* brake: max deceleration (positive); crash: max |a|   */
    float        speed_kmh;   /* brake: speed when braking started; crash: at impact */
    float        severity;    /* peak_g / threshold (>= 1.0)                          */
} event_t;

typedef struct {
    event_detection_config_t cfg;
    float    ax_f;
    bool     lpf_init;
    /* brake */
    bool     brake_armed;
    bool     brake_fired;          /* fired, waiting for recovery              */
    uint32_t brake_start_ms;
    float    brake_peak_g;
    float    brake_start_speed;
    bool     brake_has_last;
    uint32_t brake_last_event_ms;
    /* crash */
    uint8_t  crash_count;
    float    crash_peak_g;
    bool     crash_has_last;
    uint32_t crash_last_event_ms;
} event_detector_t;

/** Returns 0 on success, -1 on invalid cfg (thresholds <= 0, crash_min_samples == 0,
 *  lpf_alpha not in (0,1]). cfg NULL -> EVENT_DETECTION_CONFIG_DEFAULT. */
int event_detection_init(event_detector_t *d, const event_detection_config_t *cfg);

/**
 * Process one sample. Returns true and fills *evt when an event fires,
 * otherwise returns false and sets evt->type = EVT_NONE (evt may be NULL).
 */
bool event_detection_process(event_detector_t *d, const event_sample_t *s, event_t *evt);

/** Human-readable name, e.g. for logs: "NONE", "HARD_BRAKE", "CRASH". */
const char *event_type_str(event_type_t t);

#ifdef __cplusplus
}
#endif

#endif /* EVENT_DETECTION_H */
