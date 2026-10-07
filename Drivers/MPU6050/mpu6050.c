/**
 * @file    mpu6050.c
 * @brief   Portable MPU6050 driver core (no MCU SDK dependency).
 */
#include "mpu6050.h"

#include <string.h>

/* ------------------------------------------------------------------ Private */
#define MPU6050_WHO_AM_I_CLONE_70   0x70u
#define MPU6050_WHO_AM_I_CLONE_72   0x72u

#define MPU6050_PWR_RESET           0x80u  /* DEVICE_RESET bit            */
#define MPU6050_PWR_WAKE_PLL_X      0x01u  /* SLEEP=0, CLKSEL=1 (PLL X)   */

#define MPU6050_RESET_DELAY_MS      100u
#define MPU6050_WAKE_DELAY_MS       10u
#define MPU6050_CALIB_DELAY_MS      2u

#define MPU6050_TEMP_SENS           340.0f
#define MPU6050_TEMP_OFFSET_C       36.53f

/**
 * @brief Write a single register through the bus, mapping failure to ERR_BUS.
 */
static mpu6050_err_t write_reg(mpu6050_t *dev, uint8_t reg, uint8_t value)
{
    if (dev->bus.write(dev->bus.ctx, dev->addr, reg, &value, 1u) != 0) {
        return MPU6050_ERR_BUS;
    }
    return MPU6050_OK;
}

/**
 * @brief Return true if the WHO_AM_I value belongs to a supported chip/clone.
 */
static bool who_am_i_ok(uint8_t id)
{
    return (id == MPU6050_WHO_AM_I_VALUE) ||
           (id == MPU6050_WHO_AM_I_CLONE_70) ||
           (id == MPU6050_WHO_AM_I_CLONE_72);
}

/**
 * @brief Return LSB-per-g for an accelerometer range.
 */
static float accel_scale(mpu6050_accel_range_t r)
{
    switch (r) {
    case MPU6050_ACCEL_2G:  return 16384.0f;
    case MPU6050_ACCEL_4G:  return 8192.0f;
    case MPU6050_ACCEL_8G:  return 4096.0f;
    case MPU6050_ACCEL_16G: return 2048.0f;
    default:                return 0.0f;
    }
}

/**
 * @brief Return LSB-per-dps for a gyroscope range.
 */
static float gyro_scale(mpu6050_gyro_range_t r)
{
    switch (r) {
    case MPU6050_GYRO_250DPS:  return 131.0f;
    case MPU6050_GYRO_500DPS:  return 65.5f;
    case MPU6050_GYRO_1000DPS: return 32.8f;
    case MPU6050_GYRO_2000DPS: return 16.4f;
    default:                   return 0.0f;
    }
}

/* ---------------------------------------------------------------- Public API */
mpu6050_err_t mpu6050_init(mpu6050_t *dev, const mpu6050_bus_t *bus, uint8_t addr,
                           const mpu6050_config_t *cfg)
{
    static const mpu6050_config_t def = MPU6050_CONFIG_DEFAULT;
    mpu6050_config_t c;
    uint8_t id = 0u;
    mpu6050_err_t err;

    if (dev == NULL || bus == NULL || bus->read == NULL || bus->write == NULL ||
        bus->delay_ms == NULL) {
        return MPU6050_ERR_ARG;
    }
    c = (cfg != NULL) ? *cfg : def;
    if (accel_scale(c.accel_range) == 0.0f || gyro_scale(c.gyro_range) == 0.0f ||
        (int)c.dlpf < (int)MPU6050_DLPF_260HZ || (int)c.dlpf > (int)MPU6050_DLPF_5HZ) {
        return MPU6050_ERR_ARG;
    }

    memset(dev, 0, sizeof(*dev));
    dev->bus = *bus;
    dev->addr = addr;
    dev->cfg = c;
    dev->initialized = false;

    /* Probe the chip. */
    if (dev->bus.read(dev->bus.ctx, addr, MPU6050_REG_WHO_AM_I, &id, 1u) != 0) {
        return MPU6050_ERR_BUS;
    }
    if (!who_am_i_ok(id)) {
        return MPU6050_ERR_WHO_AM_I;
    }

    /* Reset, then wake with PLL (X gyro) clock. */
    err = write_reg(dev, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_RESET);
    if (err != MPU6050_OK) {
        return err;
    }
    dev->bus.delay_ms(dev->bus.ctx, MPU6050_RESET_DELAY_MS);

    err = write_reg(dev, MPU6050_REG_PWR_MGMT_1, MPU6050_PWR_WAKE_PLL_X);
    if (err != MPU6050_OK) {
        return err;
    }
    dev->bus.delay_ms(dev->bus.ctx, MPU6050_WAKE_DELAY_MS);

    /* Apply configuration. */
    err = write_reg(dev, MPU6050_REG_SMPLRT_DIV, c.sample_rate_div);
    if (err != MPU6050_OK) {
        return err;
    }
    err = write_reg(dev, MPU6050_REG_CONFIG, (uint8_t)((unsigned)c.dlpf & 0x07u));
    if (err != MPU6050_OK) {
        return err;
    }
    err = write_reg(dev, MPU6050_REG_GYRO_CONFIG, (uint8_t)((unsigned)c.gyro_range << 3));
    if (err != MPU6050_OK) {
        return err;
    }
    err = write_reg(dev, MPU6050_REG_ACCEL_CONFIG, (uint8_t)((unsigned)c.accel_range << 3));
    if (err != MPU6050_OK) {
        return err;
    }

    dev->accel_lsb_per_g = accel_scale(c.accel_range);
    dev->gyro_lsb_per_dps = gyro_scale(c.gyro_range);
    memset(&dev->calib, 0, sizeof(dev->calib));
    dev->initialized = true;
    return MPU6050_OK;
}

mpu6050_err_t mpu6050_read_raw(mpu6050_t *dev, mpu6050_raw_t *raw)
{
    uint8_t b[14];

    if (dev == NULL || raw == NULL) {
        return MPU6050_ERR_ARG;
    }
    if (!dev->initialized) {
        return MPU6050_ERR_NOT_INIT;
    }
    if (dev->bus.read(dev->bus.ctx, dev->addr, MPU6050_REG_ACCEL_XOUT_H, b, sizeof(b)) != 0) {
        return MPU6050_ERR_BUS;
    }

    raw->ax   = (int16_t)(uint16_t)(((uint16_t)b[0]  << 8) | b[1]);
    raw->ay   = (int16_t)(uint16_t)(((uint16_t)b[2]  << 8) | b[3]);
    raw->az   = (int16_t)(uint16_t)(((uint16_t)b[4]  << 8) | b[5]);
    raw->temp = (int16_t)(uint16_t)(((uint16_t)b[6]  << 8) | b[7]);
    raw->gx   = (int16_t)(uint16_t)(((uint16_t)b[8]  << 8) | b[9]);
    raw->gy   = (int16_t)(uint16_t)(((uint16_t)b[10] << 8) | b[11]);
    raw->gz   = (int16_t)(uint16_t)(((uint16_t)b[12] << 8) | b[13]);
    return MPU6050_OK;
}

void mpu6050_convert(const mpu6050_t *dev, const mpu6050_raw_t *raw, mpu6050_data_t *out)
{
    if (dev == NULL || raw == NULL || out == NULL) {
        return;
    }
    out->ax_g = ((float)raw->ax / dev->accel_lsb_per_g) - dev->calib.accel_bias_g[0];
    out->ay_g = ((float)raw->ay / dev->accel_lsb_per_g) - dev->calib.accel_bias_g[1];
    out->az_g = ((float)raw->az / dev->accel_lsb_per_g) - dev->calib.accel_bias_g[2];
    out->gx_dps = ((float)raw->gx / dev->gyro_lsb_per_dps) - dev->calib.gyro_bias_dps[0];
    out->gy_dps = ((float)raw->gy / dev->gyro_lsb_per_dps) - dev->calib.gyro_bias_dps[1];
    out->gz_dps = ((float)raw->gz / dev->gyro_lsb_per_dps) - dev->calib.gyro_bias_dps[2];
    out->temp_c = ((float)raw->temp / MPU6050_TEMP_SENS) + MPU6050_TEMP_OFFSET_C;
}

mpu6050_err_t mpu6050_read(mpu6050_t *dev, mpu6050_data_t *out)
{
    mpu6050_raw_t raw;
    mpu6050_err_t err;

    if (dev == NULL || out == NULL) {
        return MPU6050_ERR_ARG;
    }
    err = mpu6050_read_raw(dev, &raw);
    if (err != MPU6050_OK) {
        return err;
    }
    mpu6050_convert(dev, &raw, out);
    return MPU6050_OK;
}

mpu6050_err_t mpu6050_calibrate(mpu6050_t *dev, uint16_t samples)
{
    mpu6050_calib_t saved;
    mpu6050_raw_t raw;
    mpu6050_data_t d;
    double sum[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double n;
    uint16_t i;
    int k;
    mpu6050_err_t err;

    if (dev == NULL || samples == 0u) {
        return MPU6050_ERR_ARG;
    }
    if (!dev->initialized) {
        return MPU6050_ERR_NOT_INIT;
    }

    /* Measure with zero bias; the previous calibration stays untouched until
     * every sample has been read successfully. */
    saved = dev->calib;
    memset(&dev->calib, 0, sizeof(dev->calib));

    for (i = 0u; i < samples; i++) {
        err = mpu6050_read_raw(dev, &raw);
        if (err != MPU6050_OK) {
            dev->calib = saved;
            return err;
        }
        mpu6050_convert(dev, &raw, &d);
        sum[0] += (double)d.ax_g;
        sum[1] += (double)d.ay_g;
        sum[2] += (double)d.az_g;
        sum[3] += (double)d.gx_dps;
        sum[4] += (double)d.gy_dps;
        sum[5] += (double)d.gz_dps;
        if ((uint16_t)(i + 1u) < samples) {
            dev->bus.delay_ms(dev->bus.ctx, MPU6050_CALIB_DELAY_MS);
        }
    }

    n = (double)samples;
    for (k = 0; k < 3; k++) {
        saved.accel_bias_g[k]  = (float)(sum[k] / n);
        saved.gyro_bias_dps[k] = (float)(sum[3 + k] / n);
    }
    saved.accel_bias_g[2] -= 1.0f;  /* expected rest vector is (0, 0, +1 g) */
    dev->calib = saved;
    return MPU6050_OK;
}

void mpu6050_set_calibration(mpu6050_t *dev, const mpu6050_calib_t *calib)
{
    if (dev == NULL || calib == NULL) {
        return;
    }
    dev->calib = *calib;
}
