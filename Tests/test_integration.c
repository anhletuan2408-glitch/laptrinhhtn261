/**
 * @file    test_integration.c
 * @brief   Host end-to-end test of the sensor pipeline (no FreeRTOS).
 *
 * Re-creates the logic of Application/sensor_task.c on the host:
 *   mock I2C bus (register array) -> mpu6050 driver -> event detection
 *   simulated wheel (hall_on_pulse) -> hall driver -> speed service
 * and drives it at 100 Hz through a scenario:
 *   stationary 1 s -> accelerate to 40 km/h -> cruise -> hard brake (0.7 g, 400 ms)
 *   -> cruise at ~30 km/h -> crash (6 g, 30 ms) and stop.
 */
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "event_detection.h"
#include "hall.h"
#include "mpu6050.h"
#include "speed.h"
#include "test_common.h"

/* ------------------------------------------------------------ Scenario */
#define BASE_US         5000000ull   /* simulated clock does not start at 0 */
#define BASE_MS         5000u
#define PERIOD_MS       10u          /* SENSOR_PERIOD_MS */
#define SPEED_DIV       5u           /* SENSOR_SPEED_DIV */
#define WHEEL_CIRC_M    0.21
#define G_MPS2          9.80665
#define V_CRUISE1_MPS   (40.0 / 3.6)
#define BRAKE_G         0.7

#define T_ACCEL_START   1000.0
#define T_ACCEL_END     5000.0
#define T_BRAKE_START   8000.0
#define T_BRAKE_END     8400.0
#define T_CRASH_START   11000.0
#define T_CRASH_END     11030.0      /* 3 samples of impact */
#define T_STOP          11200.0      /* wheel standstill */
#define T_TOTAL         13500.0

#define LSB_PER_G       2048.0       /* +-16 g */
#define BIAS_AX_LSB     40.0
#define BIAS_AY_LSB     (-25.0)
#define BIAS_AZ_LSB     30.0

/** Simulated vehicle speed [m/s] at time t [ms]. */
static double v_at(double t)
{
    const double v_brake_end = V_CRUISE1_MPS - BRAKE_G * G_MPS2 * (T_BRAKE_END - T_BRAKE_START) / 1000.0;

    if (t < T_ACCEL_START) {
        return 0.0;
    }
    if (t < T_ACCEL_END) {
        return V_CRUISE1_MPS * (t - T_ACCEL_START) / (T_ACCEL_END - T_ACCEL_START);
    }
    if (t < T_BRAKE_START) {
        return V_CRUISE1_MPS;
    }
    if (t < T_BRAKE_END) {
        return V_CRUISE1_MPS - BRAKE_G * G_MPS2 * (t - T_BRAKE_START) / 1000.0;
    }
    if (t < T_CRASH_START) {
        return v_brake_end;
    }
    if (t < T_STOP) {
        return v_brake_end * (T_STOP - t) / (T_STOP - T_CRASH_START);
    }
    return 0.0;
}

/** True acceleration [g] the IMU would feel at time t [ms] (forward axis). */
static double ax_at(double t)
{
    if (t >= T_ACCEL_START && t < T_ACCEL_END) {
        return V_CRUISE1_MPS / ((T_ACCEL_END - T_ACCEL_START) / 1000.0) / G_MPS2;
    }
    if (t >= T_BRAKE_START && t < T_BRAKE_END) {
        return -BRAKE_G;
    }
    if (t >= T_CRASH_START && t < T_CRASH_END) {
        return -6.0;   /* the impact absorbs the whole deceleration */
    }
    return 0.0;
}

/* -------------------------------------------------------------- Mock bus */
static uint8_t g_regs[128];
static double  g_accel_lsb[3];   /* true raw value incl. sensor bias */
static uint32_t g_rng = 12345u;

/** Deterministic noise in [-2, +2] LSB. */
static double noise_lsb(void)
{
    g_rng = g_rng * 1664525u + 1013904223u;
    return (double)((g_rng >> 16) % 5u) - 2.0;
}

static void put_be16(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)((uint16_t)v >> 8);
    p[1] = (uint8_t)((uint16_t)v & 0xFFu);
}

static int mock_write(void *ctx, uint8_t dev, uint8_t reg, const uint8_t *data, size_t len)
{
    (void)ctx;
    (void)dev;
    if ((size_t)reg + len > sizeof(g_regs)) {
        return -1;
    }
    memcpy(&g_regs[reg], data, len);
    return 0;
}

static int mock_read(void *ctx, uint8_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    (void)ctx;
    (void)dev;
    if ((size_t)reg + len > sizeof(g_regs)) {
        return -1;
    }
    if (reg == MPU6050_REG_ACCEL_XOUT_H) {
        /* Fresh noisy measurement on every burst read. */
        for (int i = 0; i < 3; i++) {
            put_be16(&g_regs[MPU6050_REG_ACCEL_XOUT_H + 2 * i],
                     (int16_t)lround(g_accel_lsb[i] + noise_lsb()));
        }
    }
    memcpy(data, &g_regs[reg], len);
    return 0;
}

static void mock_delay(void *ctx, uint32_t ms)
{
    (void)ctx;
    (void)ms;
}

static void set_accel_g(double ax, double ay, double az)
{
    g_accel_lsb[0] = ax * LSB_PER_G + BIAS_AX_LSB;
    g_accel_lsb[1] = ay * LSB_PER_G + BIAS_AY_LSB;
    g_accel_lsb[2] = az * LSB_PER_G + BIAS_AZ_LSB;
}

/* ------------------------------------------------------- Simulation rig */
#define MAX_EVENTS 16

typedef struct {
    /* pipeline under test */
    mpu6050_t          mpu;
    hall_t             hall;
    speed_t            spd;
    event_detector_t   det;
    speed_result_t     res;
    uint32_t           cycle;
    /* wheel physics */
    double             dist_m;
    uint32_t           pulses_emitted;
    /* results */
    double             calib_ax_residual_g;
    double             calib_bias_ax_g;
    double             calib_bias_az_g;
    double             max_speed_stationary_kmh;
    double             cruise1_max_err;      /* relative */
    double             cruise2_max_err;
    int                cruise1_samples;
    int                cruise2_samples;
    event_t            events[MAX_EVENTS];
    double             event_t_ms[MAX_EVENTS];   /* relative to scenario start */
    int                n_events;
    double             t_speed_zero_ms;          /* first update with speed == 0 after stop */
    uint64_t           age_at_zero_us;           /* now - last pulse at that moment */
    int                imu_errors;
    double             sim_dist_end_m;
    double             speed_end_kmh;
} sim_t;

static sim_t g_sim;

/** Advance the wheel from t0 to t1 [ms] in 1 ms steps, firing Hall pulses. */
static void wheel_advance(sim_t *s, double t0, double t1)
{
    for (double t = t0; t < t1 - 1e-9; t += 1.0) {
        const double d0 = s->dist_m;
        const double d1 = d0 + 0.5 * (v_at(t) + v_at(t + 1.0)) * 0.001;
        while (((double)(s->pulses_emitted + 1u)) * WHEEL_CIRC_M <= d1 && d1 > d0) {
            const double target = ((double)(s->pulses_emitted + 1u)) * WHEEL_CIRC_M;
            const double frac   = (target - d0) / (d1 - d0);
            const double tp_us  = (double)BASE_US + (t + frac) * 1000.0;
            hall_on_pulse(&s->hall, (uint64_t)llround(tp_us));
            s->pulses_emitted++;
        }
        s->dist_m = d1;
    }
}

/** One cycle of the sensor task loop (mirrors Application/sensor_task.c). */
static void task_cycle(sim_t *s, uint64_t now_us, double t_ms)
{
    const uint32_t now_ms = (uint32_t)(now_us / 1000u);
    mpu6050_data_t imu;
    bool           imu_valid = false;

    if (mpu6050_read(&s->mpu, &imu) == MPU6050_OK) {
        imu_valid = true;
    } else {
        s->imu_errors++;
    }

    if ((s->cycle % SPEED_DIV) == 0u) {
        hall_snapshot_t snap;
        hall_get_snapshot(&s->hall, &snap);
        speed_update(&s->spd, &snap, now_us, &s->res);

        /* Metrics on speed updates. */
        if (t_ms < T_ACCEL_START && s->res.speed_kmh > s->max_speed_stationary_kmh) {
            s->max_speed_stationary_kmh = s->res.speed_kmh;
        }
        if (t_ms >= 5600.0 && t_ms < T_BRAKE_START) {
            const double err = fabs(s->res.speed_kmh - v_at(t_ms) * 3.6) / (v_at(t_ms) * 3.6);
            s->cruise1_samples++;
            if (err > s->cruise1_max_err) {
                s->cruise1_max_err = err;
            }
        }
        if (t_ms >= 9500.0 && t_ms < T_CRASH_START) {
            const double err = fabs(s->res.speed_kmh - v_at(t_ms) * 3.6) / (v_at(t_ms) * 3.6);
            s->cruise2_samples++;
            if (err > s->cruise2_max_err) {
                s->cruise2_max_err = err;
            }
        }
        if (t_ms > T_STOP && s->t_speed_zero_ms == 0.0 && s->res.speed_kmh == 0.0f) {
            s->t_speed_zero_ms = t_ms;
            s->age_at_zero_us  = now_us - snap.last_pulse_us;
        }
    }
    s->cycle++;

    if (imu_valid) {
        event_sample_t es;
        event_t        evt;
        es.timestamp_ms = now_ms;
        es.ax_g         = imu.ax_g;
        es.ay_g         = imu.ay_g;
        es.az_g         = imu.az_g;
        es.gx_dps       = imu.gx_dps;
        es.gy_dps       = imu.gy_dps;
        es.gz_dps       = imu.gz_dps;
        es.speed_kmh    = s->res.speed_kmh;
        if (event_detection_process(&s->det, &es, &evt) && s->n_events < MAX_EVENTS) {
            s->events[s->n_events]     = evt;
            s->event_t_ms[s->n_events] = (double)(evt.timestamp_ms - BASE_MS);
            s->n_events++;
        }
    }
}

static void run_simulation(void)
{
    sim_t *s = &g_sim;
    memset(s, 0, sizeof(*s));
    memset(g_regs, 0, sizeof(g_regs));
    g_regs[MPU6050_REG_WHO_AM_I] = MPU6050_WHO_AM_I_VALUE;

    const mpu6050_bus_t bus = { mock_write, mock_read, mock_delay, NULL };
    const speed_config_t scfg = SPEED_CONFIG_DEFAULT;

    CHECK(mpu6050_init(&s->mpu, &bus, MPU6050_ADDR_AD0_LOW, NULL) == MPU6050_OK);
    CHECK(speed_init(&s->spd, &scfg) == 0);
    CHECK(event_detection_init(&s->det, NULL) == 0);
    hall_init(&s->hall, NULL);

    /* Stationary, level, with sensor offsets: calibrate. */
    set_accel_g(0.0, 0.0, 1.0);
    CHECK(mpu6050_calibrate(&s->mpu, 200u) == MPU6050_OK);
    s->calib_bias_ax_g = (double)s->mpu.calib.accel_bias_g[0];
    s->calib_bias_az_g = (double)s->mpu.calib.accel_bias_g[2];
    {
        mpu6050_data_t d;
        CHECK(mpu6050_read(&s->mpu, &d) == MPU6050_OK);
        s->calib_ax_residual_g = (double)d.ax_g;
    }

    hall_snapshot_t snap0;
    hall_get_snapshot(&s->hall, &snap0);
    speed_reset_distance(&s->spd, snap0.pulse_count);

    double t_prev = 0.0;
    for (double t = 0.0; t <= T_TOTAL; t += (double)PERIOD_MS) {
        wheel_advance(s, t_prev, t);
        t_prev = t;
        set_accel_g(ax_at(t), (t >= T_CRASH_START && t < T_CRASH_END) ? 0.3 : 0.0, 1.0);
        task_cycle(s, BASE_US + (uint64_t)(t * 1000.0), t);
    }
    s->sim_dist_end_m = s->dist_m;
    s->speed_end_kmh  = (double)s->res.speed_kmh;
}

/* ------------------------------------------------------------------ Tests */
static void test_calibration_removes_bias(void)
{
    /* +40 LSB on ax = 0.01953 g, +30 LSB on az = 0.01465 g. */
    CHECK_NEAR(g_sim.calib_bias_ax_g, BIAS_AX_LSB / LSB_PER_G, 0.001);
    CHECK_NEAR(g_sim.calib_bias_az_g, BIAS_AZ_LSB / LSB_PER_G, 0.001);
    CHECK_NEAR(g_sim.calib_ax_residual_g, 0.0, 0.003);
}

static void test_no_speed_when_stationary(void)
{
    CHECK_NEAR(g_sim.max_speed_stationary_kmh, 0.0, 1e-6);
    CHECK(g_sim.imu_errors == 0);
}

static void test_speed_accuracy_in_cruise(void)
{
    CHECK(g_sim.cruise1_samples > 40);
    CHECK(g_sim.cruise2_samples > 20);
    CHECK(g_sim.cruise1_max_err < 0.10);
    CHECK(g_sim.cruise2_max_err < 0.10);
}

static void test_event_sequence(void)
{
    /* No events during accel / cruise: exactly one brake then one crash. */
    CHECK(g_sim.n_events == 2);
    if (g_sim.n_events < 2) {
        return;
    }
    CHECK(g_sim.events[0].type == EVT_HARD_BRAKE);
    CHECK(g_sim.events[1].type == EVT_CRASH);
}

static void test_hard_brake_event(void)
{
    if (g_sim.n_events < 1) {
        CHECK(g_sim.n_events >= 1);
        return;
    }
    const event_t *e = &g_sim.events[0];
    CHECK(g_sim.event_t_ms[0] >= T_BRAKE_START + 100.0);
    CHECK(g_sim.event_t_ms[0] <  T_BRAKE_END);
    CHECK(e->peak_g >= 0.6f && e->peak_g <= 0.8f);
    CHECK(e->speed_kmh >= 36.0f && e->speed_kmh <= 44.0f);   /* start speed ~ 40 km/h */
    CHECK(e->severity >= 1.0f);
}

static void test_crash_event(void)
{
    if (g_sim.n_events < 2) {
        CHECK(g_sim.n_events >= 2);
        return;
    }
    const event_t *e = &g_sim.events[1];
    CHECK(g_sim.event_t_ms[1] >= T_CRASH_START);
    CHECK(g_sim.event_t_ms[1] <= T_CRASH_END);
    CHECK(e->peak_g >= 5.0f);
    CHECK(e->speed_kmh > 20.0f && e->speed_kmh < 40.0f);     /* ~30 km/h at impact */
}

static void test_speed_goes_to_zero_after_stop(void)
{
    const speed_config_t sc = SPEED_CONFIG_DEFAULT;
    const uint32_t timeout_us = sc.stop_timeout_us;
    CHECK(g_sim.t_speed_zero_ms > T_STOP);
    /* Speed is forced to 0 at the first update after the timeout (<= 1 update period late). */
    CHECK(g_sim.age_at_zero_us >= (uint64_t)timeout_us);
    CHECK(g_sim.age_at_zero_us <= (uint64_t)timeout_us + 50000u);
    CHECK_NEAR(g_sim.speed_end_kmh, 0.0, 1e-6);
}

static void test_distance_accuracy(void)
{
    CHECK(g_sim.sim_dist_end_m > 60.0);
    CHECK_NEAR(g_sim.res.distance_m, g_sim.sim_dist_end_m, 0.05 * g_sim.sim_dist_end_m);
    CHECK(g_sim.res.pulse_count == g_sim.pulses_emitted);
}

int main(void)
{
    run_simulation();
    RUN_TEST(test_calibration_removes_bias);
    RUN_TEST(test_no_speed_when_stationary);
    RUN_TEST(test_speed_accuracy_in_cruise);
    RUN_TEST(test_event_sequence);
    RUN_TEST(test_hard_brake_event);
    RUN_TEST(test_crash_event);
    RUN_TEST(test_speed_goes_to_zero_after_stop);
    RUN_TEST(test_distance_accuracy);
    return TEST_REPORT();
}
