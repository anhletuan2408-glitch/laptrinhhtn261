/**
 * @file    test_mpu6050.c
 * @brief   Host unit tests for the MPU6050 driver core (mock I2C bus).
 */
#include <string.h>

#include "mpu6050.h"
#include "test_common.h"

/* --------------------------------------------------------------- Mock bus */
#define LOG_MAX 64

typedef enum { EV_WRITE, EV_DELAY } ev_kind_t;

typedef struct {
    ev_kind_t kind;
    uint8_t   reg;
    uint8_t   value;
    uint32_t  ms;
} ev_t;

typedef struct {
    uint8_t  regs[128];
    ev_t     log[LOG_MAX];
    int      log_n;
    int      read_calls;
    int      write_calls;
    int      fail_read_at;   /* fail the Nth read call (1-based), 0 = never   */
    int      fail_read_from; /* fail every read call >= N (1-based), 0 = never */
    int      fail_write_at;  /* fail the Nth write call (1-based), 0 = never  */
    uint8_t  last_addr;
    /* optional sample sequence served at ACCEL_XOUT_H */
    const uint8_t (*seq)[14];
    int      seq_n;
    int      seq_i;
} mock_t;

static int mock_write(void *ctx, uint8_t addr, uint8_t reg, const uint8_t *data, size_t len)
{
    mock_t *m = (mock_t *)ctx;
    size_t i;

    m->write_calls++;
    m->last_addr = addr;
    if (m->fail_write_at != 0 && m->write_calls == m->fail_write_at) {
        return -1;
    }
    for (i = 0; i < len; i++) {
        m->regs[(reg + i) & 0x7Fu] = data[i];
        if (m->log_n < LOG_MAX) {
            m->log[m->log_n].kind = EV_WRITE;
            m->log[m->log_n].reg = (uint8_t)(reg + i);
            m->log[m->log_n].value = data[i];
            m->log_n++;
        }
    }
    return 0;
}

static int mock_read(void *ctx, uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    mock_t *m = (mock_t *)ctx;
    size_t i;

    m->read_calls++;
    m->last_addr = addr;
    if (m->fail_read_at != 0 && m->read_calls == m->fail_read_at) {
        return -1;
    }
    if (m->fail_read_from != 0 && m->read_calls >= m->fail_read_from) {
        return -1;
    }
    if (reg == MPU6050_REG_ACCEL_XOUT_H && m->seq != NULL && len == 14u) {
        memcpy(data, m->seq[m->seq_i % m->seq_n], 14u);
        m->seq_i++;
        return 0;
    }
    for (i = 0; i < len; i++) {
        data[i] = m->regs[(reg + i) & 0x7Fu];
    }
    return 0;
}

static void mock_delay(void *ctx, uint32_t ms)
{
    mock_t *m = (mock_t *)ctx;

    if (m->log_n < LOG_MAX) {
        m->log[m->log_n].kind = EV_DELAY;
        m->log[m->log_n].ms = ms;
        m->log_n++;
    }
}

static void mock_reset(mock_t *m)
{
    memset(m, 0, sizeof(*m));
    m->regs[MPU6050_REG_WHO_AM_I] = MPU6050_WHO_AM_I_VALUE;
}

static mpu6050_bus_t make_bus(mock_t *m)
{
    mpu6050_bus_t b;

    b.write = mock_write;
    b.read = mock_read;
    b.delay_ms = mock_delay;
    b.ctx = m;
    return b;
}

/** Store a big-endian int16 into a 14-byte sample at word index `w` (0..6). */
static void put16(uint8_t *s, int w, int16_t v)
{
    s[2 * w] = (uint8_t)((uint16_t)v >> 8);
    s[2 * w + 1] = (uint8_t)((uint16_t)v & 0xFFu);
}

static void put_sample(uint8_t *s, int16_t ax, int16_t ay, int16_t az, int16_t t,
                       int16_t gx, int16_t gy, int16_t gz)
{
    put16(s, 0, ax); put16(s, 1, ay); put16(s, 2, az); put16(s, 3, t);
    put16(s, 4, gx); put16(s, 5, gy); put16(s, 6, gz);
}

/** Preload the 14 data registers of the mock with a single constant sample. */
static void mock_set_sample(mock_t *m, int16_t ax, int16_t ay, int16_t az, int16_t t,
                            int16_t gx, int16_t gy, int16_t gz)
{
    put_sample(&m->regs[MPU6050_REG_ACCEL_XOUT_H], ax, ay, az, t, gx, gy, gz);
}

/* ------------------------------------------------------------------ Tests */
static void test_init_happy_path(void)
{
    mock_t m;
    mpu6050_bus_t bus;
    mpu6050_t dev;
    mpu6050_config_t cfg = MPU6050_CONFIG_DEFAULT;
    int i;

    mock_reset(&m);
    bus = make_bus(&m);
    CHECK(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW, &cfg) == MPU6050_OK);
    CHECK(dev.initialized);
    CHECK(dev.addr == MPU6050_ADDR_AD0_LOW);
    CHECK(m.last_addr == MPU6050_ADDR_AD0_LOW);

    /* Expected exact sequence: reset, 100 ms, wake, 10 ms, then config regs. */
    CHECK(m.log_n == 8);
    if (m.log_n == 8) {
        CHECK(m.log[0].kind == EV_WRITE && m.log[0].reg == MPU6050_REG_PWR_MGMT_1 &&
              m.log[0].value == 0x80u);
        CHECK(m.log[1].kind == EV_DELAY && m.log[1].ms == 100u);
        CHECK(m.log[2].kind == EV_WRITE && m.log[2].reg == MPU6050_REG_PWR_MGMT_1 &&
              m.log[2].value == 0x01u);
        CHECK(m.log[3].kind == EV_DELAY && m.log[3].ms == 10u);
        CHECK(m.log[4].kind == EV_WRITE && m.log[4].reg == MPU6050_REG_SMPLRT_DIV &&
              m.log[4].value == 9u);
        CHECK(m.log[5].kind == EV_WRITE && m.log[5].reg == MPU6050_REG_CONFIG &&
              m.log[5].value == 3u);
        CHECK(m.log[6].kind == EV_WRITE && m.log[6].reg == MPU6050_REG_GYRO_CONFIG &&
              m.log[6].value == (1u << 3));
        CHECK(m.log[7].kind == EV_WRITE && m.log[7].reg == MPU6050_REG_ACCEL_CONFIG &&
              m.log[7].value == (3u << 3));
    }
    CHECK_NEAR(dev.accel_lsb_per_g, 2048.0, 1e-6);
    CHECK_NEAR(dev.gyro_lsb_per_dps, 65.5, 1e-6);
    for (i = 0; i < 3; i++) {
        CHECK_NEAR(dev.calib.accel_bias_g[i], 0.0, 0.0);
        CHECK_NEAR(dev.calib.gyro_bias_dps[i], 0.0, 0.0);
    }
}

static void test_init_default_cfg_and_clone_ids(void)
{
    mock_t m;
    mpu6050_bus_t bus;
    mpu6050_t dev;
    static const uint8_t ids[] = { 0x68u, 0x70u, 0x72u };
    size_t i;

    for (i = 0; i < sizeof(ids); i++) {
        mock_reset(&m);
        m.regs[MPU6050_REG_WHO_AM_I] = ids[i];
        bus = make_bus(&m);
        CHECK(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_HIGH, NULL) == MPU6050_OK);
        CHECK(dev.addr == MPU6050_ADDR_AD0_HIGH);
        CHECK(dev.cfg.accel_range == MPU6050_ACCEL_16G);
        CHECK(dev.cfg.gyro_range == MPU6050_GYRO_500DPS);
        CHECK(dev.cfg.dlpf == MPU6050_DLPF_44HZ);
        CHECK(dev.cfg.sample_rate_div == 9u);
    }
}

static void test_init_wrong_who_am_i(void)
{
    mock_t m;
    mpu6050_bus_t bus;
    mpu6050_t dev;

    mock_reset(&m);
    m.regs[MPU6050_REG_WHO_AM_I] = 0x12u;
    bus = make_bus(&m);
    CHECK(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW, NULL) == MPU6050_ERR_WHO_AM_I);
    CHECK(!dev.initialized);
    CHECK(m.write_calls == 0);  /* must not touch the chip */
}

static void test_init_bus_errors(void)
{
    mock_t m;
    mpu6050_bus_t bus;
    mpu6050_t dev;
    int w;

    /* WHO_AM_I read fails */
    mock_reset(&m);
    m.fail_read_at = 1;
    bus = make_bus(&m);
    CHECK(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW, NULL) == MPU6050_ERR_BUS);
    CHECK(!dev.initialized);

    /* Any of the 6 register writes fails */
    for (w = 1; w <= 6; w++) {
        mock_reset(&m);
        m.fail_write_at = w;
        bus = make_bus(&m);
        CHECK(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW, NULL) == MPU6050_ERR_BUS);
        CHECK(!dev.initialized);
    }
}

static void test_init_null_and_bad_args(void)
{
    mock_t m;
    mpu6050_bus_t bus, bad;
    mpu6050_t dev;
    mpu6050_config_t cfg = MPU6050_CONFIG_DEFAULT;

    mock_reset(&m);
    bus = make_bus(&m);

    CHECK(mpu6050_init(NULL, &bus, MPU6050_ADDR_AD0_LOW, NULL) == MPU6050_ERR_ARG);
    CHECK(mpu6050_init(&dev, NULL, MPU6050_ADDR_AD0_LOW, NULL) == MPU6050_ERR_ARG);

    bad = bus; bad.read = NULL;
    CHECK(mpu6050_init(&dev, &bad, MPU6050_ADDR_AD0_LOW, NULL) == MPU6050_ERR_ARG);
    bad = bus; bad.write = NULL;
    CHECK(mpu6050_init(&dev, &bad, MPU6050_ADDR_AD0_LOW, NULL) == MPU6050_ERR_ARG);
    bad = bus; bad.delay_ms = NULL;
    CHECK(mpu6050_init(&dev, &bad, MPU6050_ADDR_AD0_LOW, NULL) == MPU6050_ERR_ARG);

    cfg.accel_range = (mpu6050_accel_range_t)4;
    CHECK(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW, &cfg) == MPU6050_ERR_ARG);
    cfg = (mpu6050_config_t)MPU6050_CONFIG_DEFAULT;
    cfg.gyro_range = (mpu6050_gyro_range_t)7;
    CHECK(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW, &cfg) == MPU6050_ERR_ARG);
    cfg = (mpu6050_config_t)MPU6050_CONFIG_DEFAULT;
    cfg.dlpf = (mpu6050_dlpf_t)7;
    CHECK(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW, &cfg) == MPU6050_ERR_ARG);
    CHECK(m.read_calls == 0 && m.write_calls == 0);
}

static void test_read_before_init_and_null(void)
{
    mock_t m;
    mpu6050_bus_t bus;
    mpu6050_t dev;
    mpu6050_raw_t raw;
    mpu6050_data_t d;

    mock_reset(&m);
    bus = make_bus(&m);
    memset(&dev, 0, sizeof(dev));
    dev.bus = bus;

    CHECK(mpu6050_read_raw(&dev, &raw) == MPU6050_ERR_NOT_INIT);
    CHECK(mpu6050_read(&dev, &d) == MPU6050_ERR_NOT_INIT);
    CHECK(mpu6050_calibrate(&dev, 4u) == MPU6050_ERR_NOT_INIT);
    CHECK(m.read_calls == 0);

    CHECK(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW, NULL) == MPU6050_OK);
    CHECK(mpu6050_read_raw(NULL, &raw) == MPU6050_ERR_ARG);
    CHECK(mpu6050_read_raw(&dev, NULL) == MPU6050_ERR_ARG);
    CHECK(mpu6050_read(NULL, &d) == MPU6050_ERR_ARG);
    CHECK(mpu6050_read(&dev, NULL) == MPU6050_ERR_ARG);
    CHECK(mpu6050_calibrate(NULL, 4u) == MPU6050_ERR_ARG);
    CHECK(mpu6050_calibrate(&dev, 0u) == MPU6050_ERR_ARG);
    mpu6050_set_calibration(NULL, NULL);  /* must not crash */
    mpu6050_set_calibration(&dev, NULL);
    mpu6050_convert(NULL, NULL, NULL);
}

static void test_read_raw_big_endian_and_negative(void)
{
    mock_t m;
    mpu6050_bus_t bus;
    mpu6050_t dev;
    mpu6050_raw_t raw;
    int reads_before;

    mock_reset(&m);
    bus = make_bus(&m);
    CHECK(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW, NULL) == MPU6050_OK);

    mock_set_sample(&m, 0x1234, -2, -32768, -1000, 32767, -300, 1);
    reads_before = m.read_calls;
    CHECK(mpu6050_read_raw(&dev, &raw) == MPU6050_OK);
    CHECK(m.read_calls == reads_before + 1);  /* single burst */
    CHECK(raw.ax == 0x1234);
    CHECK(raw.ay == -2);
    CHECK(raw.az == -32768);
    CHECK(raw.temp == -1000);
    CHECK(raw.gx == 32767);
    CHECK(raw.gy == -300);
    CHECK(raw.gz == 1);

    /* byte order check on a raw register image: 0xFF 0xFE -> -2 */
    CHECK(m.regs[MPU6050_REG_ACCEL_XOUT_H + 2] == 0xFFu);
    CHECK(m.regs[MPU6050_REG_ACCEL_XOUT_H + 3] == 0xFEu);

    /* bus error */
    m.fail_read_at = m.read_calls + 1;
    CHECK(mpu6050_read_raw(&dev, &raw) == MPU6050_ERR_BUS);
}

static void test_scales_all_ranges(void)
{
    static const float accel_exp[4] = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
    static const float gyro_exp[4]  = { 131.0f, 65.5f, 32.8f, 16.4f };
    int a, g;

    for (a = 0; a < 4; a++) {
        for (g = 0; g < 4; g++) {
            mock_t m;
            mpu6050_bus_t bus;
            mpu6050_t dev;
            mpu6050_config_t cfg = MPU6050_CONFIG_DEFAULT;
            mpu6050_raw_t raw;
            mpu6050_data_t d;

            cfg.accel_range = (mpu6050_accel_range_t)a;
            cfg.gyro_range = (mpu6050_gyro_range_t)g;
            mock_reset(&m);
            bus = make_bus(&m);
            CHECK(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW, &cfg) == MPU6050_OK);
            CHECK(m.regs[MPU6050_REG_ACCEL_CONFIG] == (uint8_t)(a << 3));
            CHECK(m.regs[MPU6050_REG_GYRO_CONFIG] == (uint8_t)(g << 3));
            CHECK_NEAR(dev.accel_lsb_per_g, accel_exp[a], 1e-3);
            CHECK_NEAR(dev.gyro_lsb_per_dps, gyro_exp[g], 1e-3);

            /* exactly 1 g / 100 dps worth of counts, negative on the other axes */
            raw.ax = (int16_t)accel_exp[a];
            raw.ay = (int16_t)(-accel_exp[a] / 2.0f);
            raw.az = 0;
            raw.temp = 0;
            raw.gx = (int16_t)(gyro_exp[g] * 100.0f);
            raw.gy = (int16_t)(-gyro_exp[g] * 50.0f);
            raw.gz = 0;
            mpu6050_convert(&dev, &raw, &d);
            CHECK_NEAR(d.ax_g, 1.0, 1e-3);
            CHECK_NEAR(d.ay_g, -0.5, 1e-3);
            CHECK_NEAR(d.az_g, 0.0, 1e-6);
            CHECK_NEAR(d.gx_dps, 100.0, 0.05);
            CHECK_NEAR(d.gy_dps, -50.0, 0.05);
            CHECK_NEAR(d.gz_dps, 0.0, 1e-6);
        }
    }
}

static void test_temperature_conversion(void)
{
    mock_t m;
    mpu6050_bus_t bus;
    mpu6050_t dev;
    mpu6050_raw_t raw;
    mpu6050_data_t d;

    mock_reset(&m);
    bus = make_bus(&m);
    CHECK(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW, NULL) == MPU6050_OK);

    memset(&raw, 0, sizeof(raw));
    mpu6050_convert(&dev, &raw, &d);
    CHECK_NEAR(d.temp_c, 36.53, 1e-4);

    raw.temp = 340;
    mpu6050_convert(&dev, &raw, &d);
    CHECK_NEAR(d.temp_c, 37.53, 1e-4);

    raw.temp = -3400;
    mpu6050_convert(&dev, &raw, &d);
    CHECK_NEAR(d.temp_c, 26.53, 1e-4);

    /* through the bus */
    mock_set_sample(&m, 0, 0, 0, (int16_t)(-3624), 0, 0, 0);
    CHECK(mpu6050_read(&dev, &d) == MPU6050_OK);
    CHECK_NEAR(d.temp_c, -3624.0 / 340.0 + 36.53, 1e-4);
}

static void test_read_converts_with_calibration(void)
{
    mock_t m;
    mpu6050_bus_t bus;
    mpu6050_t dev;
    mpu6050_data_t d;
    mpu6050_calib_t c;

    mock_reset(&m);
    bus = make_bus(&m);
    CHECK(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW, NULL) == MPU6050_OK);  /* 16 g, 500 dps */

    memset(&c, 0, sizeof(c));
    c.accel_bias_g[0] = 0.02f;
    c.accel_bias_g[1] = -0.01f;
    c.accel_bias_g[2] = 0.05f;
    c.gyro_bias_dps[0] = 1.5f;
    c.gyro_bias_dps[1] = -0.5f;
    c.gyro_bias_dps[2] = 0.25f;
    mpu6050_set_calibration(&dev, &c);

    mock_set_sample(&m, 2048 / 10, 0, 2048 + 100, 0, 655, 0, 0);
    CHECK(mpu6050_read(&dev, &d) == MPU6050_OK);
    CHECK_NEAR(d.ax_g, 204.0 / 2048.0 - 0.02, 1e-5);
    CHECK_NEAR(d.ay_g, 0.01, 1e-5);
    CHECK_NEAR(d.az_g, 2148.0 / 2048.0 - 0.05, 1e-5);
    CHECK_NEAR(d.gx_dps, 10.0 - 1.5, 1e-4);
    CHECK_NEAR(d.gy_dps, 0.5, 1e-5);
    CHECK_NEAR(d.gz_dps, -0.25, 1e-5);
}

static void test_calibrate_math(void)
{
    mock_t m;
    mpu6050_bus_t bus;
    mpu6050_t dev;
    mpu6050_data_t d;
    mpu6050_config_t cfg = MPU6050_CONFIG_DEFAULT;
    uint8_t seq[4][14];
    int delays_before, delays = 0, i;

    cfg.accel_range = MPU6050_ACCEL_2G;   /* 16384 LSB/g */
    cfg.gyro_range = MPU6050_GYRO_250DPS; /* 131 LSB/dps */
    mock_reset(&m);
    bus = make_bus(&m);
    CHECK(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW, &cfg) == MPU6050_OK);

    /* Four samples with a small offset on every axis (az near 1 g, 2 g range). */
    put_sample(seq[0], 100, -300, 17000, 0, 250, -100, 60);
    put_sample(seq[1], 228, -356, 16800, 0, 274, -162, 70);
    put_sample(seq[2], 164, -328, 16900, 0, 262, -131, 65);
    put_sample(seq[3], 172, -344, 16860, 0, 238, -131, 71);
    {
        /* compute exact expected means from the raw counts */
        double sax = 0, say = 0, saz = 0, sgx = 0, sgy = 0, sgz = 0;
        static const int16_t ax[4] = { 100, 228, 164, 172 };
        static const int16_t ay[4] = { -300, -356, -328, -344 };
        static const int16_t az[4] = { 17000, 16800, 16900, 16860 };
        static const int16_t gx[4] = { 250, 274, 262, 238 };
        static const int16_t gy[4] = { -100, -162, -131, -131 };
        static const int16_t gz[4] = { 60, 70, 65, 71 };
        for (i = 0; i < 4; i++) {
            sax += ax[i]; say += ay[i]; saz += az[i];
            sgx += gx[i]; sgy += gy[i]; sgz += gz[i];
        }
        m.seq = (const uint8_t (*)[14])seq;
        m.seq_n = 4;
        m.seq_i = 0;

        delays_before = m.log_n;
        CHECK(mpu6050_calibrate(&dev, 4u) == MPU6050_OK);
        for (i = delays_before; i < m.log_n; i++) {
            if (m.log[i].kind == EV_DELAY) {
                CHECK(m.log[i].ms == 2u);
                delays++;
            }
        }
        CHECK(delays == 3);  /* between samples, not after the last one */

        CHECK_NEAR(dev.calib.accel_bias_g[0], sax / 4.0 / 16384.0, 1e-6);
        CHECK_NEAR(dev.calib.accel_bias_g[1], say / 4.0 / 16384.0, 1e-6);
        CHECK_NEAR(dev.calib.accel_bias_g[2], saz / 4.0 / 16384.0 - 1.0, 1e-6);
        CHECK_NEAR(dev.calib.gyro_bias_dps[0], sgx / 4.0 / 131.0, 1e-5);
        CHECK_NEAR(dev.calib.gyro_bias_dps[1], sgy / 4.0 / 131.0, 1e-5);
        CHECK_NEAR(dev.calib.gyro_bias_dps[2], sgz / 4.0 / 131.0, 1e-5);

        /* After calibration the mean rest vector reads (0, 0, 1) and gyro 0. */
        m.seq_i = 0;
        {
            double mx = 0, my = 0, mz = 0, rx = 0, ry = 0, rz = 0;
            for (i = 0; i < 4; i++) {
                CHECK(mpu6050_read(&dev, &d) == MPU6050_OK);
                mx += d.ax_g; my += d.ay_g; mz += d.az_g;
                rx += d.gx_dps; ry += d.gy_dps; rz += d.gz_dps;
            }
            CHECK_NEAR(mx / 4.0, 0.0, 1e-5);
            CHECK_NEAR(my / 4.0, 0.0, 1e-5);
            CHECK_NEAR(mz / 4.0, 1.0, 1e-5);
            CHECK_NEAR(rx / 4.0, 0.0, 1e-4);
            CHECK_NEAR(ry / 4.0, 0.0, 1e-4);
            CHECK_NEAR(rz / 4.0, 0.0, 1e-4);
        }
    }
}

static void test_calibrate_ignores_previous_calibration(void)
{
    mock_t m;
    mpu6050_bus_t bus;
    mpu6050_t dev;
    mpu6050_calib_t c;

    mock_reset(&m);
    bus = make_bus(&m);
    CHECK(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW, NULL) == MPU6050_OK);  /* 2048 LSB/g */

    memset(&c, 0, sizeof(c));
    c.accel_bias_g[0] = 5.0f;
    c.gyro_bias_dps[2] = 7.0f;
    mpu6050_set_calibration(&dev, &c);

    mock_set_sample(&m, 0, 0, 2048, 0, 0, 0, 0);  /* perfect rest */
    CHECK(mpu6050_calibrate(&dev, 3u) == MPU6050_OK);
    CHECK_NEAR(dev.calib.accel_bias_g[0], 0.0, 1e-6);
    CHECK_NEAR(dev.calib.accel_bias_g[2], 0.0, 1e-6);
    CHECK_NEAR(dev.calib.gyro_bias_dps[2], 0.0, 1e-6);
}

static void test_calibrate_failure_keeps_old_calib(void)
{
    mock_t m;
    mpu6050_bus_t bus;
    mpu6050_t dev;
    mpu6050_calib_t c;
    mpu6050_data_t d;
    int fail_sample;

    for (fail_sample = 1; fail_sample <= 5; fail_sample++) {
        mock_reset(&m);
        bus = make_bus(&m);
        CHECK(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW, NULL) == MPU6050_OK);

        memset(&c, 0, sizeof(c));
        c.accel_bias_g[0] = 0.1f;
        c.accel_bias_g[1] = 0.2f;
        c.accel_bias_g[2] = 0.3f;
        c.gyro_bias_dps[0] = 1.0f;
        c.gyro_bias_dps[1] = 2.0f;
        c.gyro_bias_dps[2] = 3.0f;
        mpu6050_set_calibration(&dev, &c);

        mock_set_sample(&m, 500, 600, 2600, 0, 70, 80, 90);
        m.fail_read_from = m.read_calls + fail_sample;  /* Nth read of the run fails */
        CHECK(mpu6050_calibrate(&dev, 5u) == MPU6050_ERR_BUS);

        CHECK(memcmp(&dev.calib, &c, sizeof(c)) == 0);
        CHECK(dev.initialized);

        /* old calibration is still effectively applied */
        m.fail_read_from = 0;
        CHECK(mpu6050_read(&dev, &d) == MPU6050_OK);
        CHECK_NEAR(d.ax_g, 500.0 / 2048.0 - 0.1, 1e-5);
        CHECK_NEAR(d.gz_dps, 90.0 / 65.5 - 3.0, 1e-4);
    }
}

static void test_calibrate_does_not_leave_half_applied(void)
{
    /* On failure after several good samples calib must equal the old one even
     * though the intermediate measurements used a zeroed calibration. */
    mock_t m;
    mpu6050_bus_t bus;
    mpu6050_t dev;
    mpu6050_calib_t c, before;

    mock_reset(&m);
    bus = make_bus(&m);
    CHECK(mpu6050_init(&dev, &bus, MPU6050_ADDR_AD0_LOW, NULL) == MPU6050_OK);
    memset(&c, 0, sizeof(c));
    c.accel_bias_g[2] = -0.25f;
    mpu6050_set_calibration(&dev, &c);
    before = dev.calib;

    mock_set_sample(&m, 1, 2, 3, 4, 5, 6, 7);
    m.fail_read_from = m.read_calls + 4;
    CHECK(mpu6050_calibrate(&dev, 10u) == MPU6050_ERR_BUS);
    CHECK(memcmp(&dev.calib, &before, sizeof(before)) == 0);
}

int main(void)
{
    RUN_TEST(test_init_happy_path);
    RUN_TEST(test_init_default_cfg_and_clone_ids);
    RUN_TEST(test_init_wrong_who_am_i);
    RUN_TEST(test_init_bus_errors);
    RUN_TEST(test_init_null_and_bad_args);
    RUN_TEST(test_read_before_init_and_null);
    RUN_TEST(test_read_raw_big_endian_and_negative);
    RUN_TEST(test_scales_all_ranges);
    RUN_TEST(test_temperature_conversion);
    RUN_TEST(test_read_converts_with_calibration);
    RUN_TEST(test_calibrate_math);
    RUN_TEST(test_calibrate_ignores_previous_calibration);
    RUN_TEST(test_calibrate_failure_keeps_old_calib);
    RUN_TEST(test_calibrate_does_not_leave_half_applied);
    return TEST_REPORT();
}
