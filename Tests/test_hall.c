/**
 * @file    test_hall.c
 * @brief   Host unit tests for the Hall wheel sensor driver.
 */
#include <stdint.h>
#include <string.h>

#include "hall.h"
#include "test_common.h"

static void test_init_default_cfg(void)
{
    hall_t h;
    hall_snapshot_t s;

    memset(&h, 0xA5, sizeof h);
    hall_init(&h, NULL);
    CHECK(h.cfg.min_interval_us == 2000u);
    hall_get_snapshot(&h, &s);
    CHECK(s.pulse_count == 0u);
    CHECK(s.last_period_us == 0u);
    CHECK(s.last_pulse_us == 0u);
    CHECK(s.rejected_count == 0u);
}

static void test_init_custom_cfg(void)
{
    hall_t h;
    hall_config_t cfg = { 500u };

    hall_init(&h, &cfg);
    CHECK(h.cfg.min_interval_us == 500u);
}

static void test_first_pulse(void)
{
    hall_t h;
    hall_snapshot_t s;

    hall_init(&h, NULL);
    hall_on_pulse(&h, 123456u);
    hall_get_snapshot(&h, &s);
    CHECK(s.pulse_count == 1u);
    CHECK(s.last_period_us == 0u);
    CHECK(s.last_pulse_us == 123456u);
    CHECK(s.rejected_count == 0u);
}

static void test_period(void)
{
    hall_t h;
    hall_snapshot_t s;

    hall_init(&h, NULL);
    hall_on_pulse(&h, 1000000u);
    hall_on_pulse(&h, 1100000u);
    hall_get_snapshot(&h, &s);
    CHECK(s.pulse_count == 2u);
    CHECK(s.last_period_us == 100000u);
    CHECK(s.last_pulse_us == 1100000u);

    hall_on_pulse(&h, 1150000u);
    hall_get_snapshot(&h, &s);
    CHECK(s.pulse_count == 3u);
    CHECK(s.last_period_us == 50000u);
    CHECK(s.last_pulse_us == 1150000u);
}

static void test_debounce_reject(void)
{
    hall_t h;
    hall_snapshot_t s;

    hall_init(&h, NULL);              /* 2000 us */
    hall_on_pulse(&h, 10000u);
    hall_on_pulse(&h, 11999u);        /* 1999 us -> rejected */
    hall_get_snapshot(&h, &s);
    CHECK(s.pulse_count == 1u);
    CHECK(s.rejected_count == 1u);
    CHECK(s.last_pulse_us == 10000u);
    CHECK(s.last_period_us == 0u);

    hall_on_pulse(&h, 12000u);        /* exactly min interval -> accepted */
    hall_get_snapshot(&h, &s);
    CHECK(s.pulse_count == 2u);
    CHECK(s.rejected_count == 1u);
    CHECK(s.last_period_us == 2000u);
    CHECK(s.last_pulse_us == 12000u);

    /* A rejected pulse is measured against the last ACCEPTED one. */
    hall_on_pulse(&h, 13000u);
    hall_on_pulse(&h, 14000u);        /* 2000 after 12000 -> accepted */
    hall_get_snapshot(&h, &s);
    CHECK(s.pulse_count == 3u);
    CHECK(s.rejected_count == 2u);
    CHECK(s.last_pulse_us == 14000u);
}

static void test_backwards_timestamp(void)
{
    hall_t h;
    hall_snapshot_t s;

    hall_init(&h, NULL);
    hall_on_pulse(&h, 500000u);
    hall_on_pulse(&h, 600000u);
    hall_on_pulse(&h, 100000u);       /* backwards -> rejected */
    hall_get_snapshot(&h, &s);
    CHECK(s.pulse_count == 2u);
    CHECK(s.rejected_count == 1u);
    CHECK(s.last_pulse_us == 600000u);
    CHECK(s.last_period_us == 100000u);

    hall_on_pulse(&h, 700000u);       /* normal operation continues */
    hall_get_snapshot(&h, &s);
    CHECK(s.pulse_count == 3u);
    CHECK(s.last_period_us == 100000u);
}

static void test_period_saturates(void)
{
    hall_t h;
    hall_snapshot_t s;

    hall_init(&h, NULL);
    hall_on_pulse(&h, 1000u);
    hall_on_pulse(&h, 1000u + (uint64_t)UINT32_MAX + 5000u);
    hall_get_snapshot(&h, &s);
    CHECK(s.pulse_count == 2u);
    CHECK(s.last_period_us == UINT32_MAX);
    CHECK(s.last_pulse_us == 1000u + (uint64_t)UINT32_MAX + 5000u);
}

static void test_64bit_timestamp(void)
{
    hall_t h;
    hall_snapshot_t s;
    const uint64_t base = 0x100000000ULL * 3u + 17u;   /* beyond 32 bits */

    hall_init(&h, NULL);
    hall_on_pulse(&h, base);
    hall_on_pulse(&h, base + 250000u);
    hall_get_snapshot(&h, &s);
    CHECK(s.last_pulse_us == base + 250000u);
    CHECK(s.last_period_us == 250000u);
}

static void test_reset(void)
{
    hall_t h;
    hall_snapshot_t s;
    hall_config_t cfg = { 1000u };

    hall_init(&h, &cfg);
    hall_on_pulse(&h, 10000u);
    hall_on_pulse(&h, 10100u);        /* rejected */
    hall_on_pulse(&h, 20000u);
    hall_reset(&h);
    hall_get_snapshot(&h, &s);
    CHECK(s.pulse_count == 0u);
    CHECK(s.last_period_us == 0u);
    CHECK(s.last_pulse_us == 0u);
    CHECK(s.rejected_count == 0u);
    CHECK(h.cfg.min_interval_us == 1000u);   /* cfg kept */

    /* Behaves as a fresh sensor: first pulse stores timestamp only. */
    hall_on_pulse(&h, 5000u);
    hall_get_snapshot(&h, &s);
    CHECK(s.pulse_count == 1u);
    CHECK(s.last_period_us == 0u);
    CHECK(s.last_pulse_us == 5000u);
}

static void test_snapshot_is_copy(void)
{
    hall_t h;
    hall_snapshot_t s1, s2;

    hall_init(&h, NULL);
    hall_on_pulse(&h, 10000u);
    hall_on_pulse(&h, 60000u);
    hall_get_snapshot(&h, &s1);
    hall_on_pulse(&h, 110000u);
    hall_get_snapshot(&h, &s2);
    CHECK(s1.pulse_count == 2u);
    CHECK(s1.last_pulse_us == 60000u);
    CHECK(s2.pulse_count == 3u);
    CHECK(s2.last_pulse_us == 110000u);
}

static void test_null_args(void)
{
    hall_t h;
    hall_snapshot_t s;

    hall_init(NULL, NULL);
    hall_on_pulse(NULL, 1u);
    hall_reset(NULL);
    hall_init(&h, NULL);
    hall_get_snapshot(&h, NULL);
    memset(&s, 0xFF, sizeof s);
    hall_get_snapshot(NULL, &s);
    CHECK(s.pulse_count == 0u);
    CHECK(s.last_pulse_us == 0u);
}

int main(void)
{
    RUN_TEST(test_init_default_cfg);
    RUN_TEST(test_init_custom_cfg);
    RUN_TEST(test_first_pulse);
    RUN_TEST(test_period);
    RUN_TEST(test_debounce_reject);
    RUN_TEST(test_backwards_timestamp);
    RUN_TEST(test_period_saturates);
    RUN_TEST(test_64bit_timestamp);
    RUN_TEST(test_reset);
    RUN_TEST(test_snapshot_is_copy);
    RUN_TEST(test_null_args);
    return TEST_REPORT();
}
