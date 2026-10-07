/**
 * @file    test_event_detection.c
 * @brief   Host unit tests for Services/EventDetection.
 */
#include <stdint.h>
#include <string.h>

#include "event_detection.h"
#include "test_common.h"

#define STEP_MS 10u

/** Events recorded while feeding samples. */
typedef struct {
    int     count;
    event_t first;
    event_t last;
} rec_t;

/**
 * Feed n samples spaced STEP_MS apart starting at *t (advanced as we go).
 * speed ramps from speed0 by dspeed per sample.
 */
static void feed(event_detector_t *d, uint32_t *t, int n,
                 float ax, float ay, float az,
                 float speed0, float dspeed, rec_t *r)
{
    for (int i = 0; i < n; i++) {
        event_sample_t s;
        event_t        e;
        memset(&s, 0, sizeof(s));
        e.type         = EVT_CRASH;   /* sentinel: must be overwritten */
        s.timestamp_ms = *t;
        s.ax_g         = ax;
        s.ay_g         = ay;
        s.az_g         = az;
        s.speed_kmh    = speed0 + dspeed * (float)i;
        bool fired     = event_detection_process(d, &s, &e);
        if (!fired) {
            CHECK(e.type == EVT_NONE);
        }
        if (fired && r != NULL) {
            if (r->count == 0) {
                r->first = e;
            }
            r->last = e;
            r->count++;
        }
        *t += STEP_MS;
    }
}

/** Feed constant-speed samples until *t reaches t_end (exclusive). */
static void feed_until(event_detector_t *d, uint32_t *t, uint32_t t_end,
                       float ax, float az, float speed, rec_t *r)
{
    int n = (int)((uint32_t)(t_end - *t) / STEP_MS);
    feed(d, t, n, ax, 0.0f, az, speed, 0.0f, r);
}

static void quiet(event_detector_t *d, uint32_t *t, int n, float speed, rec_t *r)
{
    feed(d, t, n, 0.0f, 0.0f, 1.0f, speed, 0.0f, r);
}

static void init_default(event_detector_t *d)
{
    CHECK(event_detection_init(d, NULL) == 0);
}

/* ------------------------------------------------------------------ */

static void test_init_validation(void)
{
    event_detector_t         d;
    event_detection_config_t c = EVENT_DETECTION_CONFIG_DEFAULT;

    CHECK(event_detection_init(NULL, NULL) == -1);
    CHECK(event_detection_init(&d, NULL) == 0);
    CHECK_NEAR(d.cfg.brake_threshold_g, 0.45, 1e-6);
    CHECK(d.cfg.brake_min_duration_ms == 150u);
    CHECK(d.cfg.crash_min_samples == 2u);
    CHECK(!d.lpf_init && !d.brake_armed && !d.brake_fired && d.crash_count == 0);

    CHECK(event_detection_init(&d, &c) == 0);

    c = (event_detection_config_t)EVENT_DETECTION_CONFIG_DEFAULT;
    c.brake_threshold_g = 0.0f;
    CHECK(event_detection_init(&d, &c) == -1);
    c.brake_threshold_g = -0.1f;
    CHECK(event_detection_init(&d, &c) == -1);

    c = (event_detection_config_t)EVENT_DETECTION_CONFIG_DEFAULT;
    c.crash_threshold_g = 0.0f;
    CHECK(event_detection_init(&d, &c) == -1);

    c = (event_detection_config_t)EVENT_DETECTION_CONFIG_DEFAULT;
    c.crash_min_samples = 0;
    CHECK(event_detection_init(&d, &c) == -1);

    c = (event_detection_config_t)EVENT_DETECTION_CONFIG_DEFAULT;
    c.lpf_alpha = 0.0f;
    CHECK(event_detection_init(&d, &c) == -1);
    c.lpf_alpha = 1.01f;
    CHECK(event_detection_init(&d, &c) == -1);
    c.lpf_alpha = -0.5f;
    CHECK(event_detection_init(&d, &c) == -1);
    c.lpf_alpha = NAN;
    CHECK(event_detection_init(&d, &c) == -1);
    c.lpf_alpha = 1.0f;                       /* upper bound is inclusive */
    CHECK(event_detection_init(&d, &c) == 0);

    c = (event_detection_config_t)EVENT_DETECTION_CONFIG_DEFAULT;
    c.brake_threshold_g = NAN;
    CHECK(event_detection_init(&d, &c) == -1);

    c = (event_detection_config_t)EVENT_DETECTION_CONFIG_DEFAULT;
    c.brake_min_speed_kmh = -1.0f;
    CHECK(event_detection_init(&d, &c) == -1);
    c.brake_min_speed_kmh = 0.0f;             /* zero min speed is allowed */
    CHECK(event_detection_init(&d, &c) == 0);
}

static void test_null_args(void)
{
    event_detector_t d;
    event_sample_t   s;
    event_t          e;

    init_default(&d);
    memset(&s, 0, sizeof(s));
    e.type = EVT_CRASH;
    CHECK(!event_detection_process(NULL, &s, &e));
    CHECK(e.type == EVT_NONE);
    e.type = EVT_CRASH;
    CHECK(!event_detection_process(&d, NULL, &e));
    CHECK(e.type == EVT_NONE);
    CHECK(!event_detection_process(&d, NULL, NULL));
}

static void test_type_str(void)
{
    CHECK(strcmp(event_type_str(EVT_NONE), "NONE") == 0);
    CHECK(strcmp(event_type_str(EVT_HARD_BRAKE), "HARD_BRAKE") == 0);
    CHECK(strcmp(event_type_str(EVT_CRASH), "CRASH") == 0);
    CHECK(event_type_str((event_type_t)99) != NULL);
}

static void test_quiet_driving_no_events(void)
{
    event_detector_t d;
    rec_t            r = {0};
    uint32_t         t = 1000;

    init_default(&d);
    /* 5 s of cruising with +-0.05 g jitter, 1 g on z */
    for (int i = 0; i < 250; i++) {
        feed(&d, &t, 1, 0.05f, 0.02f, 1.0f, 50.0f, 0.0f, &r);
        feed(&d, &t, 1, -0.05f, -0.02f, 1.0f, 50.0f, 0.0f, &r);
    }
    CHECK(r.count == 0);
    /* Normal acceleration (+0.3 g) and a gentle brake (-0.3 g) are not events. */
    feed(&d, &t, 100, 0.3f, 0.0f, 1.0f, 50.0f, 0.0f, &r);
    feed(&d, &t, 100, -0.3f, 0.0f, 1.0f, 50.0f, 0.0f, &r);
    CHECK(r.count == 0);
}

static void test_lpf_behaviour(void)
{
    event_detector_t d;
    rec_t            r = {0};
    uint32_t         t = 0;

    init_default(&d);
    CHECK(!d.lpf_init);
    feed(&d, &t, 1, -2.0f, 0.0f, 1.0f, 0.0f, 0.0f, &r);   /* speed 0: no brake arming */
    CHECK(d.lpf_init);
    CHECK_NEAR(d.ax_f, -2.0, 1e-6);                       /* initialised, not blended with 0 */
    feed(&d, &t, 1, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, &r);
    CHECK_NEAR(d.ax_f, -1.4, 1e-5);                       /* -2 + 0.3*(0+2) */
    feed(&d, &t, 1, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, &r);
    CHECK_NEAR(d.ax_f, -0.98, 1e-5);

    /* step response: ax_f(k) = A*(1 - 0.7^k) after starting from 0 */
    init_default(&d);
    t = 0;
    quiet(&d, &t, 1, 0.0f, &r);
    CHECK_NEAR(d.ax_f, 0.0, 1e-6);
    feed(&d, &t, 3, 0.5f, 0.0f, 1.0f, 0.0f, 0.0f, &r);
    CHECK_NEAR(d.ax_f, 0.5 * (1.0 - pow(0.7, 3)), 1e-5);

    /* alpha == 1 -> no filtering */
    event_detection_config_t c = EVENT_DETECTION_CONFIG_DEFAULT;
    c.lpf_alpha = 1.0f;
    CHECK(event_detection_init(&d, &c) == 0);
    t = 0;
    quiet(&d, &t, 1, 0.0f, &r);
    feed(&d, &t, 1, -0.7f, 0.0f, 1.0f, 0.0f, 0.0f, &r);
    CHECK_NEAR(d.ax_f, -0.7, 1e-6);

    /* Oscillation is attenuated: +-0.8 g alternating -> ripple ~0.14 g,
     * well below the 0.45 g threshold, so no brake event even at speed. */
    init_default(&d);
    t = 0;
    memset(&r, 0, sizeof(r));
    quiet(&d, &t, 1, 60.0f, &r);
    float peak = 0.0f;
    for (int i = 0; i < 100; i++) {
        feed(&d, &t, 1, (i & 1) ? 0.8f : -0.8f, 0.0f, 1.0f, 60.0f, 0.0f, &r);
        if (i > 20 && fabsf(d.ax_f) > peak) {
            peak = fabsf(d.ax_f);
        }
    }
    CHECK(peak < 0.2f);
    CHECK(r.count == 0);
}

static void test_hard_brake_fires_once(void)
{
    event_detector_t d;
    rec_t            r = {0};
    uint32_t         t = 1000;

    init_default(&d);
    quiet(&d, &t, 20, 60.0f, &r);
    const uint32_t T = t;   /* first braking sample */

    /* Braking at -0.8 g; speed falls 0.5 km/h per sample from 60.
     * ax_f(k) = -0.8*(1-0.7^k): -0.24, -0.408, -0.526 -> armed on 3rd sample (T+20),
     * speed at that sample = 59.0. Fires when t - (T+20) >= 150 -> T+170 (18th sample). */
    feed(&d, &t, 17, -0.8f, 0.0f, 1.0f, 60.0f, -0.5f, &r);
    CHECK(r.count == 0);
    CHECK(d.brake_armed);
    t = T + 17 * STEP_MS;
    {
        rec_t r2 = {0};
        feed(&d, &t, 1, -0.8f, 0.0f, 1.0f, 60.0f - 0.5f * 17.0f, 0.0f, &r2);
        CHECK(r2.count == 1);
        CHECK(r2.first.type == EVT_HARD_BRAKE);
        CHECK(r2.first.timestamp_ms == T + 170u);
        CHECK_NEAR(r2.first.peak_g, 0.8 * (1.0 - pow(0.7, 18)), 1e-4);
        CHECK(r2.first.peak_g > 0.0f);
        CHECK_NEAR(r2.first.speed_kmh, 59.0, 1e-4);
        CHECK_NEAR(r2.first.severity, r2.first.peak_g / 0.45, 1e-4);
        CHECK(r2.first.severity >= 1.0f);
        CHECK(!d.brake_armed && d.brake_fired);
    }

    /* Continued braking does not fire again. */
    feed(&d, &t, 500, -0.9f, 0.0f, 1.0f, 30.0f, 0.0f, &r);
    CHECK(r.count == 0);
}

static void test_brake_peak_tracks_most_negative(void)
{
    event_detector_t d;
    rec_t            r = {0};
    uint32_t         t = 0;

    init_default(&d);
    quiet(&d, &t, 5, 80.0f, &r);
    /* ramp the decel from -0.6 g to -1.2 g and hold, so the peak is at the end. */
    feed(&d, &t, 10, -0.6f, 0.0f, 1.0f, 80.0f, 0.0f, &r);
    CHECK(r.count == 0);
    feed(&d, &t, 30, -1.2f, 0.0f, 1.0f, 70.0f, 0.0f, &r);
    CHECK(r.count == 1);
    CHECK(r.first.type == EVT_HARD_BRAKE);
    CHECK_NEAR(r.first.speed_kmh, 80.0, 1e-4);          /* speed when braking started */
    CHECK(r.first.peak_g > 0.6f);
    CHECK(r.first.peak_g <= 1.2f);
    CHECK(r.first.severity > 1.0f);
}

static void test_short_brake_spike_no_event(void)
{
    event_detector_t d;
    rec_t            r = {0};
    uint32_t         t = 0;

    init_default(&d);
    quiet(&d, &t, 20, 60.0f, &r);
    /* 50 ms of hard braking: ax_f stays below -0.45 only ~40 ms after arming. */
    feed(&d, &t, 5, -0.8f, 0.0f, 1.0f, 60.0f, 0.0f, &r);
    quiet(&d, &t, 100, 60.0f, &r);
    CHECK(r.count == 0);
    CHECK(!d.brake_armed);
    CHECK(!d.brake_fired);

    /* The disarmed detector can still fire normally afterwards. */
    feed(&d, &t, 40, -0.8f, 0.0f, 1.0f, 60.0f, 0.0f, &r);
    CHECK(r.count == 1);
}

static void test_brake_low_speed_no_event(void)
{
    event_detector_t d;
    rec_t            r = {0};
    uint32_t         t = 0;

    init_default(&d);
    quiet(&d, &t, 10, 3.0f, &r);
    feed(&d, &t, 100, -0.9f, 0.0f, 1.0f, 3.0f, 0.0f, &r);
    CHECK(r.count == 0);
    CHECK(!d.brake_armed);

    /* Just below the 5 km/h limit: no event; at the limit: event. */
    init_default(&d);
    t = 0;
    quiet(&d, &t, 10, 4.9f, &r);
    feed(&d, &t, 50, -0.9f, 0.0f, 1.0f, 4.9f, 0.0f, &r);
    CHECK(r.count == 0);
    quiet(&d, &t, 20, 4.9f, &r);   /* recover */
    feed(&d, &t, 50, -0.9f, 0.0f, 1.0f, 5.0f, 0.0f, &r);
    CHECK(r.count == 1);
    CHECK_NEAR(r.first.speed_kmh, 5.0, 1e-5);

    /* Braking already in progress when the speed condition becomes true
     * arms at that moment: event 150 ms after arming. */
    init_default(&d);
    t = 0;
    memset(&r, 0, sizeof(r));
    quiet(&d, &t, 10, 4.0f, &r);
    feed(&d, &t, 50, -0.9f, 0.0f, 1.0f, 4.0f, 0.0f, &r);
    CHECK(r.count == 0);
    const uint32_t T = t;
    feed(&d, &t, 30, -0.9f, 0.0f, 1.0f, 8.0f, 0.0f, &r);
    CHECK(r.count == 1);
    CHECK(r.first.timestamp_ms == T + 150u);
    CHECK_NEAR(r.first.speed_kmh, 8.0, 1e-5);
}

static void test_brake_recovery_and_cooldown(void)
{
    event_detector_t d;
    rec_t            r = {0};
    uint32_t         t = 0;

    /* A: brake, recover, brake again inside the cooldown -> no 2nd event;
     *    a brake after the cooldown -> 2nd event. */
    init_default(&d);
    quiet(&d, &t, 20, 60.0f, &r);
    feed(&d, &t, 40, -0.8f, 0.0f, 1.0f, 60.0f, 0.0f, &r);
    CHECK(r.count == 1);
    const uint32_t t1 = r.first.timestamp_ms;
    r.count = 0;

    quiet(&d, &t, 30, 60.0f, &r);                   /* recovery */
    CHECK(!d.brake_fired);
    feed(&d, &t, 50, -0.8f, 0.0f, 1.0f, 60.0f, 0.0f, &r);   /* within cooldown */
    CHECK(r.count == 0);
    CHECK((uint32_t)(t - t1) < 2000u);
    quiet(&d, &t, 30, 60.0f, &r);
    feed_until(&d, &t, t1 + 2500u, 0.0f, 1.0f, 60.0f, &r);  /* well past cooldown */
    CHECK(r.count == 0);
    feed(&d, &t, 40, -0.8f, 0.0f, 1.0f, 60.0f, 0.0f, &r);
    CHECK(r.count == 1);
    CHECK((uint32_t)(r.first.timestamp_ms - t1) >= 2000u);

    /* B: continuous braking (no recovery) never fires a 2nd event. */
    init_default(&d);
    t = 0;
    memset(&r, 0, sizeof(r));
    quiet(&d, &t, 20, 60.0f, &r);
    feed(&d, &t, 1000, -0.8f, 0.0f, 1.0f, 60.0f, 0.0f, &r);   /* 10 s */
    CHECK(r.count == 1);

    /* C: recovery inside the cooldown, braking resumes and is sustained:
     *    arming is only allowed once cooldown elapsed, exactly at t1+2000. */
    init_default(&d);
    t = 0;
    memset(&r, 0, sizeof(r));
    quiet(&d, &t, 20, 60.0f, &r);
    feed(&d, &t, 40, -0.8f, 0.0f, 1.0f, 60.0f, 0.0f, &r);
    CHECK(r.count == 1);
    const uint32_t t1c = r.first.timestamp_ms;
    r.count = 0;
    quiet(&d, &t, 20, 60.0f, &r);
    feed_until(&d, &t, t1c + 1990u, -0.8f, 1.0f, 60.0f, &r);  /* last sample at +1980 */
    CHECK(r.count == 0);
    CHECK(!d.brake_armed);
    feed(&d, &t, 1, -0.8f, 0.0f, 1.0f, 60.0f, 0.0f, &r);      /* sample at t1+1990 */
    CHECK(!d.brake_armed);
    feed(&d, &t, 1, -0.8f, 0.0f, 1.0f, 60.0f, 0.0f, &r);      /* sample at t1+2000 */
    CHECK(d.brake_armed);
    feed(&d, &t, 30, -0.8f, 0.0f, 1.0f, 60.0f, 0.0f, &r);
    CHECK(r.count == 1);
    CHECK(r.first.timestamp_ms == t1c + 2150u);
}

static void test_crash_two_samples(void)
{
    event_detector_t d;
    rec_t            r = {0};
    uint32_t         t = 5000;

    init_default(&d);
    quiet(&d, &t, 20, 45.0f, &r);

    /* Two consecutive samples >= 3 g: fires on the second one. */
    feed(&d, &t, 1, 3.5f, 0.0f, 1.0f, 45.0f, 0.0f, &r);     /* |a| = 3.64 */
    CHECK(r.count == 0);
    const uint32_t t_second = t;
    feed(&d, &t, 1, 0.0f, -4.0f, 1.0f, 40.0f, 0.0f, &r);    /* |a| = 4.12 */
    CHECK(r.count == 1);
    CHECK(r.first.type == EVT_CRASH);
    CHECK(r.first.timestamp_ms == t_second);
    CHECK_NEAR(r.first.peak_g, sqrt(17.0), 1e-4);            /* max of the two */
    CHECK_NEAR(r.first.speed_kmh, 40.0, 1e-5);               /* speed at impact */
    CHECK_NEAR(r.first.severity, sqrt(17.0) / 3.0, 1e-4);
    CHECK(r.first.severity >= 1.0f);

    /* Threshold is inclusive: exactly 3.0 g on two samples. */
    init_default(&d);
    t = 0;
    memset(&r, 0, sizeof(r));
    quiet(&d, &t, 5, 0.0f, &r);
    feed(&d, &t, 2, 3.0f, 0.0f, 0.0f, 0.0f, 0.0f, &r);
    CHECK(r.count == 1);
    CHECK_NEAR(r.first.severity, 1.0, 1e-6);

    /* Sustained impact after firing: only one event. */
    feed(&d, &t, 50, 4.0f, 0.0f, 0.0f, 0.0f, 0.0f, &r);
    CHECK(r.count == 1);
}

static void test_crash_single_spike_ignored(void)
{
    event_detector_t d;
    rec_t            r = {0};
    uint32_t         t = 0;

    init_default(&d);
    quiet(&d, &t, 20, 45.0f, &r);
    feed(&d, &t, 1, 6.0f, 0.0f, 1.0f, 45.0f, 0.0f, &r);   /* one-sample spike */
    quiet(&d, &t, 20, 45.0f, &r);
    CHECK(r.count == 0);
    CHECK(d.crash_count == 0);

    /* Two spikes separated by a quiet sample are not "consecutive". */
    feed(&d, &t, 1, 6.0f, 0.0f, 1.0f, 45.0f, 0.0f, &r);
    quiet(&d, &t, 1, 45.0f, &r);
    feed(&d, &t, 1, 6.0f, 0.0f, 1.0f, 45.0f, 0.0f, &r);
    quiet(&d, &t, 20, 45.0f, &r);
    CHECK(r.count == 0);

    /* Configurable sample count: 3 required. */
    event_detection_config_t c = EVENT_DETECTION_CONFIG_DEFAULT;
    c.crash_min_samples = 3;
    CHECK(event_detection_init(&d, &c) == 0);
    t = 0;
    feed(&d, &t, 2, 5.0f, 0.0f, 1.0f, 45.0f, 0.0f, &r);
    CHECK(r.count == 0);
    feed(&d, &t, 1, 5.0f, 0.0f, 1.0f, 45.0f, 0.0f, &r);
    CHECK(r.count == 1);
}

static void test_crash_cooldown(void)
{
    event_detector_t d;
    rec_t            r = {0};
    uint32_t         t = 0;

    init_default(&d);
    quiet(&d, &t, 10, 45.0f, &r);
    feed(&d, &t, 2, 4.0f, 0.0f, 1.0f, 45.0f, 0.0f, &r);
    CHECK(r.count == 1);
    const uint32_t t1 = r.first.timestamp_ms;
    r.count = 0;

    /* A second impact 2 s later is suppressed. */
    feed_until(&d, &t, t1 + 2000u, 0.0f, 1.0f, 0.0f, &r);
    feed(&d, &t, 2, 4.0f, 0.0f, 1.0f, 0.0f, 0.0f, &r);
    CHECK(r.count == 0);

    /* Just before the cooldown ends (t1 + 4990): still suppressed. */
    feed_until(&d, &t, t1 + 4980u, 0.0f, 1.0f, 0.0f, &r);
    feed(&d, &t, 2, 4.0f, 0.0f, 1.0f, 0.0f, 0.0f, &r);      /* t1+4980, t1+4990 */
    CHECK(r.count == 0);
    quiet(&d, &t, 1, 0.0f, &r);                              /* t1+5000 */
    CHECK(r.count == 0);

    /* After the cooldown it fires again (needs a fresh run of 2 samples). */
    feed(&d, &t, 2, 4.0f, 0.0f, 1.0f, 0.0f, 0.0f, &r);
    CHECK(r.count == 1);
    CHECK((uint32_t)(r.first.timestamp_ms - t1) >= 5000u);
}

static void test_crash_cancels_armed_brake(void)
{
    event_detector_t d;
    rec_t            r = {0};
    uint32_t         t = 0;

    init_default(&d);
    quiet(&d, &t, 20, 60.0f, &r);
    feed(&d, &t, 5, -0.8f, 0.0f, 1.0f, 60.0f, 0.0f, &r);   /* brake armed, not yet fired */
    CHECK(r.count == 0);
    CHECK(d.brake_armed);

    /* Frontal impact: ax = -4 g for two samples. */
    feed(&d, &t, 1, -4.0f, 0.0f, 1.0f, 55.0f, 0.0f, &r);
    CHECK(r.count == 0);
    CHECK(d.brake_armed);
    feed(&d, &t, 1, -4.0f, 0.0f, 1.0f, 40.0f, 0.0f, &r);
    CHECK(r.count == 1);
    CHECK(r.first.type == EVT_CRASH);
    CHECK_NEAR(r.first.speed_kmh, 40.0, 1e-5);
    CHECK(!d.brake_armed);

    /* No hard-brake event for the same episode, even if decel persists. */
    feed(&d, &t, 100, -0.8f, 0.0f, 1.0f, 10.0f, 0.0f, &r);
    CHECK(r.count == 1);
    CHECK(r.last.type == EVT_CRASH);

    /* After a recovery the brake detector works again. */
    quiet(&d, &t, 30, 60.0f, &r);
    feed(&d, &t, 40, -0.8f, 0.0f, 1.0f, 60.0f, 0.0f, &r);
    CHECK(r.count == 2);
    CHECK(r.last.type == EVT_HARD_BRAKE);
}

static void test_crash_unarmed_suppresses_brake(void)
{
    /* Lateral impact: brake is NOT armed at the crash sample, but the
     * following forward decel (ax_f dragged negative) must not report a
     * redundant HARD_BRAKE. */
    event_detector_t d;
    rec_t            r = {0};
    uint32_t         t = 0;

    init_default(&d);
    quiet(&d, &t, 20, 50.0f, &r);
    CHECK(!d.brake_armed);
    feed(&d, &t, 2, 0.0f, 4.0f, 1.0f, 50.0f, 0.0f, &r);   /* ay = +4 g */
    CHECK(r.count == 1);
    CHECK(r.first.type == EVT_CRASH);
    CHECK(!d.brake_armed);

    /* 1 s of -2 g forward decel (|a| = 2.24 g < crash threshold). */
    feed(&d, &t, 100, -2.0f, 0.0f, 1.0f, 50.0f, 0.0f, &r);
    CHECK(r.count == 1);
    CHECK(r.last.type == EVT_CRASH);

    /* Recover, then a normal hard brake is reported again. */
    quiet(&d, &t, 30, 50.0f, &r);
    feed(&d, &t, 40, -0.8f, 0.0f, 1.0f, 50.0f, 0.0f, &r);
    CHECK(r.count == 2);
    CHECK(r.last.type == EVT_HARD_BRAKE);
}

static void test_brake_event_then_crash(void)
{
    /* Brake fires on one call and a crash on a later call: both reported. */
    event_detector_t d;
    rec_t            r = {0};
    uint32_t         t = 0;

    init_default(&d);
    quiet(&d, &t, 20, 60.0f, &r);
    feed(&d, &t, 30, -0.8f, 0.0f, 1.0f, 60.0f, 0.0f, &r);
    CHECK(r.count == 1);
    CHECK(r.last.type == EVT_HARD_BRAKE);
    feed(&d, &t, 2, -5.0f, 0.0f, 1.0f, 30.0f, 0.0f, &r);
    CHECK(r.count == 2);
    CHECK(r.last.type == EVT_CRASH);
}

static void test_timestamp_wraparound(void)
{
    event_detector_t d;
    rec_t            r = {0};
    uint32_t         t = UINT32_MAX - 100u;   /* wraps after ~10 samples */

    /* Hard brake across the wrap point: same timing as without the wrap. */
    init_default(&d);
    quiet(&d, &t, 5, 60.0f, &r);
    const uint32_t T = t;
    feed(&d, &t, 40, -0.8f, 0.0f, 1.0f, 60.0f, 0.0f, &r);
    CHECK(r.count == 1);
    CHECK(r.first.timestamp_ms == (uint32_t)(T + 170u));
    CHECK(r.first.timestamp_ms < T);                         /* the clock really wrapped */

    /* Brake cooldown across wrap: event at wrapped t1; recover; brake inside cooldown. */
    const uint32_t t1 = r.first.timestamp_ms;
    r.count = 0;
    quiet(&d, &t, 30, 60.0f, &r);
    feed(&d, &t, 50, -0.8f, 0.0f, 1.0f, 60.0f, 0.0f, &r);
    CHECK(r.count == 0);
    quiet(&d, &t, 30, 60.0f, &r);
    feed_until(&d, &t, t1 + 2500u, 0.0f, 1.0f, 60.0f, &r);
    feed(&d, &t, 40, -0.8f, 0.0f, 1.0f, 60.0f, 0.0f, &r);
    CHECK(r.count == 1);

    /* Crash cooldown across wrap. */
    init_default(&d);
    t = UINT32_MAX - 1000u;
    memset(&r, 0, sizeof(r));
    quiet(&d, &t, 10, 45.0f, &r);
    feed(&d, &t, 2, 4.0f, 0.0f, 1.0f, 45.0f, 0.0f, &r);
    CHECK(r.count == 1);
    const uint32_t tc = r.first.timestamp_ms;
    r.count = 0;
    feed_until(&d, &t, tc + 2000u, 0.0f, 1.0f, 45.0f, &r);   /* crosses wrap */
    CHECK(t < tc);                                           /* wrapped */
    feed(&d, &t, 2, 4.0f, 0.0f, 1.0f, 45.0f, 0.0f, &r);
    CHECK(r.count == 0);
    feed_until(&d, &t, tc + 5000u, 0.0f, 1.0f, 45.0f, &r);
    feed(&d, &t, 2, 4.0f, 0.0f, 1.0f, 45.0f, 0.0f, &r);
    CHECK(r.count == 1);
}

static void test_evt_null_allowed(void)
{
    event_detector_t d;
    uint32_t         t = 0;
    int              fired = 0;

    init_default(&d);
    for (int i = 0; i < 20; i++) {
        event_sample_t s;
        memset(&s, 0, sizeof(s));
        s.timestamp_ms = t;
        s.az_g         = 1.0f;
        s.speed_kmh    = 60.0f;
        CHECK(!event_detection_process(&d, &s, NULL));
        t += STEP_MS;
    }
    for (int i = 0; i < 40; i++) {
        event_sample_t s;
        memset(&s, 0, sizeof(s));
        s.timestamp_ms = t;
        s.ax_g         = -0.8f;
        s.az_g         = 1.0f;
        s.speed_kmh    = 60.0f;
        if (event_detection_process(&d, &s, NULL)) {
            fired++;
        }
        t += STEP_MS;
    }
    CHECK(fired == 1);
    for (int i = 0; i < 2; i++) {
        event_sample_t s;
        memset(&s, 0, sizeof(s));
        s.timestamp_ms = t;
        s.ax_g         = 5.0f;
        s.az_g         = 1.0f;
        if (event_detection_process(&d, &s, NULL)) {
            fired++;
        }
        t += STEP_MS;
    }
    CHECK(fired == 2);
}

int main(void)
{
    RUN_TEST(test_init_validation);
    RUN_TEST(test_null_args);
    RUN_TEST(test_type_str);
    RUN_TEST(test_quiet_driving_no_events);
    RUN_TEST(test_lpf_behaviour);
    RUN_TEST(test_hard_brake_fires_once);
    RUN_TEST(test_brake_peak_tracks_most_negative);
    RUN_TEST(test_short_brake_spike_no_event);
    RUN_TEST(test_brake_low_speed_no_event);
    RUN_TEST(test_brake_recovery_and_cooldown);
    RUN_TEST(test_crash_two_samples);
    RUN_TEST(test_crash_single_spike_ignored);
    RUN_TEST(test_crash_cooldown);
    RUN_TEST(test_crash_cancels_armed_brake);
    RUN_TEST(test_crash_unarmed_suppresses_brake);
    RUN_TEST(test_brake_event_then_crash);
    RUN_TEST(test_timestamp_wraparound);
    RUN_TEST(test_evt_null_allowed);
    return TEST_REPORT();
}
