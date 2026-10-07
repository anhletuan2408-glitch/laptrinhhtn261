/**
 * @file    test_speed.c
 * @brief   Host unit tests for the speed / distance service.
 */
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "hall.h"
#include "speed.h"
#include "test_common.h"

/* circ 2.0 m, 1 pulse/rev, 2 s timeout, no filtering */
static speed_config_t cfg_plain(void)
{
    speed_config_t c = { 2.0f, 1u, 2000000u, 1.0f };
    return c;
}

static void test_init_invalid(void)
{
    speed_t s;
    speed_config_t c;

    CHECK(speed_init(NULL, NULL) == -1);

    c = cfg_plain(); c.wheel_circumference_m = 0.0f;
    CHECK(speed_init(&s, &c) == -1);
    c = cfg_plain(); c.wheel_circumference_m = -1.0f;
    CHECK(speed_init(&s, &c) == -1);
    c = cfg_plain(); c.wheel_circumference_m = NAN;
    CHECK(speed_init(&s, &c) == -1);
    c = cfg_plain(); c.pulses_per_rev = 0u;
    CHECK(speed_init(&s, &c) == -1);
    c = cfg_plain(); c.ema_alpha = 0.0f;
    CHECK(speed_init(&s, &c) == -1);
    c = cfg_plain(); c.ema_alpha = 1.01f;
    CHECK(speed_init(&s, &c) == -1);
    c = cfg_plain(); c.ema_alpha = -0.5f;
    CHECK(speed_init(&s, &c) == -1);
    c = cfg_plain(); c.ema_alpha = NAN;
    CHECK(speed_init(&s, &c) == -1);
}

static void test_init_valid_and_default(void)
{
    speed_t s;
    speed_config_t c = { 1.9f, 4u, 1000000u, 0.25f };

    CHECK(speed_init(&s, &c) == 0);
    CHECK_NEAR(s.dist_per_pulse_m, 1.9f / 4.0f, 1e-6);
    CHECK_NEAR(s.v_mps, 0.0, 1e-9);
    CHECK(s.pulse_base == 0u);

    CHECK(speed_init(&s, NULL) == 0);
    CHECK_NEAR(s.cfg.wheel_circumference_m, 0.21, 1e-6);
    CHECK(s.cfg.pulses_per_rev == 1u);
    CHECK(s.cfg.stop_timeout_us == 2000000u);
    CHECK_NEAR(s.cfg.ema_alpha, 0.5, 1e-6);
    CHECK_NEAR(s.dist_per_pulse_m, 0.21, 1e-6);

    /* Failed init must not require a valid object afterwards but must not crash. */
    c.ema_alpha = 2.0f;
    CHECK(speed_init(&s, &c) == -1);
}

static void test_zero_before_two_pulses(void)
{
    speed_t s;
    hall_t h;
    hall_snapshot_t snap;
    speed_result_t r;
    speed_config_t c = cfg_plain();

    CHECK(speed_init(&s, &c) == 0);
    hall_init(&h, NULL);

    /* No pulse at all. */
    hall_get_snapshot(&h, &snap);
    speed_update(&s, &snap, 1000000u, &r);
    CHECK_NEAR(r.speed_mps, 0.0, 1e-9);
    CHECK_NEAR(r.speed_kmh, 0.0, 1e-9);
    CHECK_NEAR(r.accel_mps2, 0.0, 1e-9);
    CHECK_NEAR(r.distance_m, 0.0, 1e-9);
    CHECK(r.pulse_count == 0u);

    /* Exactly one pulse: period unknown. */
    hall_on_pulse(&h, 1100000u);
    hall_get_snapshot(&h, &snap);
    speed_update(&s, &snap, 1150000u, &r);
    CHECK_NEAR(r.speed_mps, 0.0, 1e-9);
    CHECK(r.pulse_count == 1u);
    CHECK_NEAR(r.distance_m, 2.0, 1e-6);
}

static void test_constant_speed(void)
{
    speed_t s;
    hall_t h;
    hall_snapshot_t snap;
    speed_result_t r;
    speed_config_t c = cfg_plain();
    uint64_t t;

    CHECK(speed_init(&s, &c) == 0);
    hall_init(&h, NULL);

    /* Pulse every 100 ms -> 2 m / 0.1 s = 20 m/s = 72 km/h. */
    for (t = 100000u; t <= 1000000u; t += 100000u) {
        hall_on_pulse(&h, t);
        hall_get_snapshot(&h, &snap);
        speed_update(&s, &snap, t, &r);
    }
    CHECK_NEAR(r.speed_mps, 20.0, 1e-3);
    CHECK_NEAR(r.speed_kmh, 72.0, 1e-2);
    CHECK_NEAR(r.accel_mps2, 0.0, 1e-3);
    CHECK(r.pulse_count == 10u);
    CHECK_NEAR(r.distance_m, 20.0, 1e-4);

    /* Polling between pulses (since < period) keeps the same speed. */
    hall_get_snapshot(&h, &snap);
    speed_update(&s, &snap, 1050000u, &r);
    CHECK_NEAR(r.speed_mps, 20.0, 1e-3);
    speed_update(&s, &snap, 1100000u, &r);   /* since == period: no decay yet */
    CHECK_NEAR(r.speed_mps, 20.0, 1e-3);
}

static void test_pulses_per_rev(void)
{
    speed_t s;
    hall_t h;
    hall_snapshot_t snap;
    speed_result_t r;
    speed_config_t c = { 2.0f, 4u, 2000000u, 1.0f };   /* 0.5 m per pulse */

    CHECK(speed_init(&s, &c) == 0);
    hall_init(&h, NULL);
    hall_on_pulse(&h, 100000u);
    hall_on_pulse(&h, 150000u);   /* 50 ms -> 0.5 / 0.05 = 10 m/s */
    hall_get_snapshot(&h, &snap);
    speed_update(&s, &snap, 150000u, &r);
    CHECK_NEAR(r.speed_mps, 10.0, 1e-3);
    CHECK_NEAR(r.distance_m, 1.0, 1e-5);
}

static void test_stop_timeout(void)
{
    speed_t s;
    hall_t h;
    hall_snapshot_t snap;
    speed_result_t r;
    speed_config_t c = { 2.0f, 1u, 500000u, 1.0f };   /* 0.5 s timeout */

    CHECK(speed_init(&s, &c) == 0);
    hall_init(&h, NULL);
    hall_on_pulse(&h, 100000u);
    hall_on_pulse(&h, 200000u);
    hall_get_snapshot(&h, &snap);
    speed_update(&s, &snap, 200000u, &r);
    CHECK_NEAR(r.speed_mps, 20.0, 1e-3);

    /* 1 us before timeout: decayed (2 m / 0.499999 s ~ 4 m/s), not zero. */
    speed_update(&s, &snap, 200000u + 499999u, &r);
    CHECK_NEAR(r.speed_mps, 4.0, 1e-3);

    /* At the timeout: zero immediately. */
    speed_update(&s, &snap, 200000u + 500000u, &r);
    CHECK_NEAR(r.speed_mps, 0.0, 1e-9);
    CHECK_NEAR(r.speed_kmh, 0.0, 1e-9);

    /* Distance is kept after stopping. */
    CHECK_NEAR(r.distance_m, 4.0, 1e-5);

    /* Speed resumes with new pulses. */
    hall_on_pulse(&h, 900000u);
    hall_get_snapshot(&h, &snap);
    speed_update(&s, &snap, 900000u, &r);
    CHECK_NEAR(r.speed_mps, 2.0 / 0.7, 1e-3);
}

static void test_decay_when_pulses_stop(void)
{
    speed_t s;
    hall_t h;
    hall_snapshot_t snap;
    speed_result_t r;
    speed_config_t c = cfg_plain();

    CHECK(speed_init(&s, &c) == 0);
    hall_init(&h, NULL);
    hall_on_pulse(&h, 100000u);
    hall_on_pulse(&h, 200000u);   /* period 100 ms -> 20 m/s */
    hall_get_snapshot(&h, &snap);

    speed_update(&s, &snap, 200000u, &r);
    CHECK_NEAR(r.speed_mps, 20.0, 1e-3);

    speed_update(&s, &snap, 400000u, &r);   /* 200 ms since pulse -> 10 m/s */
    CHECK_NEAR(r.speed_mps, 10.0, 1e-3);
    CHECK(r.accel_mps2 < 0.0f);
    CHECK_NEAR(r.accel_mps2, (10.0 - 20.0) / 0.2, 1e-2);

    speed_update(&s, &snap, 1200000u, &r);  /* 1 s since pulse -> 2 m/s */
    CHECK_NEAR(r.speed_mps, 2.0, 1e-3);

    /* Decay is monotonic. */
    {
        float prev = r.speed_mps;
        uint64_t t;
        for (t = 1300000u; t < 2200000u; t += 100000u) {
            speed_update(&s, &snap, t, &r);
            CHECK(r.speed_mps <= prev);
            prev = r.speed_mps;
        }
    }
}

static void test_ema_convergence(void)
{
    speed_t s;
    hall_t h;
    hall_snapshot_t snap;
    speed_result_t r;
    speed_config_t c = { 2.0f, 1u, 2000000u, 0.5f };
    uint64_t t;
    int i;

    CHECK(speed_init(&s, &c) == 0);
    hall_init(&h, NULL);
    hall_on_pulse(&h, 100000u);
    hall_on_pulse(&h, 200000u);
    hall_get_snapshot(&h, &snap);

    /* Constant raw = 20 m/s: v follows 10, 15, 17.5, ... */
    speed_update(&s, &snap, 200000u, &r);
    CHECK_NEAR(r.speed_mps, 10.0, 1e-4);
    speed_update(&s, &snap, 210000u, &r);
    CHECK_NEAR(r.speed_mps, 15.0, 1e-4);
    speed_update(&s, &snap, 220000u, &r);
    CHECK_NEAR(r.speed_mps, 17.5, 1e-4);

    t = 220000u;
    for (i = 0; i < 40; i++) {
        t += 1000u;
        speed_update(&s, &snap, t, &r);
    }
    CHECK_NEAR(r.speed_mps, 20.0, 1e-3);
    CHECK(r.speed_mps <= 20.0001f);

    /* Zero raw drops to 0 immediately, no filtering. */
    speed_update(&s, &snap, 200000u + 2000000u, &r);
    CHECK_NEAR(r.speed_mps, 0.0, 1e-9);
}

static void test_accel_sign(void)
{
    speed_t s;
    hall_t h;
    hall_snapshot_t snap;
    speed_result_t r;
    speed_config_t c = cfg_plain();

    CHECK(speed_init(&s, &c) == 0);
    hall_init(&h, NULL);

    /* First update: accel is 0 (no previous update). */
    hall_on_pulse(&h, 100000u);
    hall_on_pulse(&h, 200000u);
    hall_get_snapshot(&h, &snap);
    speed_update(&s, &snap, 200000u, &r);
    CHECK_NEAR(r.speed_mps, 20.0, 1e-3);
    CHECK_NEAR(r.accel_mps2, 0.0, 1e-9);

    /* Speed up: period 100 ms -> 50 ms, v 20 -> 40 m/s over 50 ms = +400 m/s^2. */
    hall_on_pulse(&h, 250000u);
    hall_get_snapshot(&h, &snap);
    speed_update(&s, &snap, 250000u, &r);
    CHECK_NEAR(r.speed_mps, 40.0, 1e-3);
    CHECK(r.accel_mps2 > 0.0f);
    CHECK_NEAR(r.accel_mps2, 400.0, 0.5);

    /* Slow down: period 50 ms -> 100 ms, v 40 -> 20 m/s over 100 ms = -200 m/s^2. */
    hall_on_pulse(&h, 350000u);
    hall_get_snapshot(&h, &snap);
    speed_update(&s, &snap, 350000u, &r);
    CHECK_NEAR(r.speed_mps, 20.0, 1e-3);
    CHECK(r.accel_mps2 < 0.0f);
    CHECK_NEAR(r.accel_mps2, -200.0, 0.5);

    /* dt == 0: accel is 0, speed unchanged. */
    speed_update(&s, &snap, 350000u, &r);
    CHECK_NEAR(r.accel_mps2, 0.0, 1e-9);
    CHECK_NEAR(r.speed_mps, 20.0, 1e-3);
}

static void test_clock_backwards(void)
{
    speed_t s;
    hall_t h;
    hall_snapshot_t snap;
    speed_result_t r;
    speed_config_t c = cfg_plain();

    CHECK(speed_init(&s, &c) == 0);
    hall_init(&h, NULL);
    hall_on_pulse(&h, 1000000u);
    hall_on_pulse(&h, 1100000u);
    hall_get_snapshot(&h, &snap);
    speed_update(&s, &snap, 1100000u, &r);

    /* now < last_pulse and now < previous update: no huge values. */
    speed_update(&s, &snap, 1000000u, &r);
    CHECK_NEAR(r.speed_mps, 20.0, 1e-3);
    CHECK_NEAR(r.accel_mps2, 0.0, 1e-9);
    CHECK(isfinite(r.distance_m));
}

static void test_distance_and_reset(void)
{
    speed_t s;
    hall_t h;
    hall_snapshot_t snap;
    speed_result_t r;
    speed_config_t c = cfg_plain();
    uint64_t t;

    CHECK(speed_init(&s, &c) == 0);
    hall_init(&h, NULL);

    for (t = 100000u; t <= 500000u; t += 100000u) {
        hall_on_pulse(&h, t);
    }
    hall_get_snapshot(&h, &snap);
    speed_update(&s, &snap, 500000u, &r);
    CHECK(r.pulse_count == 5u);
    CHECK_NEAR(r.distance_m, 10.0, 1e-5);

    /* Trip reset at the current pulse count. */
    speed_reset_distance(&s, snap.pulse_count);
    speed_update(&s, &snap, 510000u, &r);
    CHECK_NEAR(r.distance_m, 0.0, 1e-9);
    CHECK(r.pulse_count == 5u);

    hall_on_pulse(&h, 600000u);
    hall_on_pulse(&h, 700000u);
    hall_get_snapshot(&h, &snap);
    speed_update(&s, &snap, 700000u, &r);
    CHECK_NEAR(r.distance_m, 4.0, 1e-5);
    CHECK(r.pulse_count == 7u);
}

static void test_counter_goes_backwards(void)
{
    speed_t s;
    hall_t h;
    hall_snapshot_t snap;
    speed_result_t r;
    speed_config_t c = cfg_plain();
    uint64_t t;

    CHECK(speed_init(&s, &c) == 0);
    hall_init(&h, NULL);
    for (t = 100000u; t <= 800000u; t += 100000u) {
        hall_on_pulse(&h, t);
    }
    hall_get_snapshot(&h, &snap);
    speed_reset_distance(&s, snap.pulse_count);   /* base = 8 */
    speed_update(&s, &snap, 800000u, &r);

    /* hall_reset() makes pulse_count (< pulse_base) restart from 0. */
    hall_reset(&h);
    hall_get_snapshot(&h, &snap);
    speed_update(&s, &snap, 850000u, &r);
    CHECK_NEAR(r.distance_m, 0.0, 1e-9);
    CHECK(r.pulse_count == 0u);
    CHECK_NEAR(r.speed_mps, 0.0, 1e-9);

    /* Distance counts again from the new origin, no huge value. */
    hall_on_pulse(&h, 900000u);
    hall_on_pulse(&h, 1000000u);
    hall_on_pulse(&h, 1100000u);
    hall_get_snapshot(&h, &snap);
    speed_update(&s, &snap, 1100000u, &r);
    CHECK(r.pulse_count == 3u);
    CHECK_NEAR(r.distance_m, 6.0, 1e-5);
}

static void test_null_args(void)
{
    speed_t s;
    hall_snapshot_t snap;
    speed_result_t r;

    memset(&snap, 0, sizeof snap);
    CHECK(speed_init(&s, NULL) == 0);
    speed_update(NULL, &snap, 0u, &r);
    speed_update(&s, NULL, 0u, &r);
    speed_update(&s, &snap, 1000u, NULL);   /* out may be NULL */
    speed_reset_distance(NULL, 0u);
    CHECK(1);
}

int main(void)
{
    RUN_TEST(test_init_invalid);
    RUN_TEST(test_init_valid_and_default);
    RUN_TEST(test_zero_before_two_pulses);
    RUN_TEST(test_constant_speed);
    RUN_TEST(test_pulses_per_rev);
    RUN_TEST(test_stop_timeout);
    RUN_TEST(test_decay_when_pulses_stop);
    RUN_TEST(test_ema_convergence);
    RUN_TEST(test_accel_sign);
    RUN_TEST(test_clock_backwards);
    RUN_TEST(test_distance_and_reset);
    RUN_TEST(test_counter_goes_backwards);
    RUN_TEST(test_null_args);
    return TEST_REPORT();
}
