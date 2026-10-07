/**
 * @file    mpu6050.h
 * @brief   Portable MPU6050 (6-axis IMU) driver.
 *
 * The driver core does not depend on any MCU SDK. All bus access goes through
 * mpu6050_bus_t, so the same code runs on ESP32 (see port/mpu6050_port_esp32.c)
 * and on a PC for unit tests (mock bus).
 *
 * Axis convention (sensor mounted flat, X pointing to vehicle front):
 *   +X = forward, +Y = left, +Z = up.  At rest: ax ~ 0, ay ~ 0, az ~ +1 g.
 */
#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- Constants */
#define MPU6050_ADDR_AD0_LOW    0x68u
#define MPU6050_ADDR_AD0_HIGH   0x69u

/* Register map (subset used by the driver) */
#define MPU6050_REG_SMPLRT_DIV    0x19u
#define MPU6050_REG_CONFIG        0x1Au
#define MPU6050_REG_GYRO_CONFIG   0x1Bu
#define MPU6050_REG_ACCEL_CONFIG  0x1Cu
#define MPU6050_REG_INT_PIN_CFG   0x37u
#define MPU6050_REG_INT_ENABLE    0x38u
#define MPU6050_REG_INT_STATUS    0x3Au
#define MPU6050_REG_ACCEL_XOUT_H  0x3Bu  /* 14-byte burst: accel(6) temp(2) gyro(6) */
#define MPU6050_REG_PWR_MGMT_1    0x6Bu
#define MPU6050_REG_WHO_AM_I      0x75u

#define MPU6050_WHO_AM_I_VALUE    0x68u

/* --------------------------------------------------------------- Error code */
typedef enum {
    MPU6050_OK = 0,
    MPU6050_ERR_ARG,        /* NULL pointer / invalid parameter        */
    MPU6050_ERR_BUS,        /* bus read/write callback returned != 0   */
    MPU6050_ERR_WHO_AM_I,   /* unexpected WHO_AM_I (wrong chip/wiring)  */
    MPU6050_ERR_NOT_INIT    /* call mpu6050_init() first                */
} mpu6050_err_t;

/* ----------------------------------------------------------- Configuration */
typedef enum {
    MPU6050_ACCEL_2G  = 0,   /* 16384 LSB/g */
    MPU6050_ACCEL_4G  = 1,   /*  8192 LSB/g */
    MPU6050_ACCEL_8G  = 2,   /*  4096 LSB/g */
    MPU6050_ACCEL_16G = 3    /*  2048 LSB/g */
} mpu6050_accel_range_t;

typedef enum {
    MPU6050_GYRO_250DPS  = 0,  /* 131.0 LSB/dps */
    MPU6050_GYRO_500DPS  = 1,  /*  65.5 LSB/dps */
    MPU6050_GYRO_1000DPS = 2,  /*  32.8 LSB/dps */
    MPU6050_GYRO_2000DPS = 3   /*  16.4 LSB/dps */
} mpu6050_gyro_range_t;

/* Digital low-pass filter, value written to CONFIG[2:0] */
typedef enum {
    MPU6050_DLPF_260HZ = 0,
    MPU6050_DLPF_184HZ = 1,
    MPU6050_DLPF_94HZ  = 2,
    MPU6050_DLPF_44HZ  = 3,
    MPU6050_DLPF_21HZ  = 4,
    MPU6050_DLPF_10HZ  = 5,
    MPU6050_DLPF_5HZ   = 6
} mpu6050_dlpf_t;

typedef struct {
    mpu6050_accel_range_t accel_range;
    mpu6050_gyro_range_t  gyro_range;
    mpu6050_dlpf_t        dlpf;
    uint8_t               sample_rate_div; /* rate = 1kHz / (1 + div) when DLPF on */
} mpu6050_config_t;

/** Defaults for a car black box: +-16 g (crash peaks), +-500 dps, DLPF 44 Hz, 100 Hz. */
#define MPU6050_CONFIG_DEFAULT { MPU6050_ACCEL_16G, MPU6050_GYRO_500DPS, MPU6050_DLPF_44HZ, 9u }

/* ------------------------------------------------------------ Bus interface */
/**
 * Platform bus callbacks. Each returns 0 on success, non-zero on failure.
 * read/write operate on consecutive registers starting at `reg`.
 */
typedef struct {
    int  (*write)(void *ctx, uint8_t dev_addr, uint8_t reg, const uint8_t *data, size_t len);
    int  (*read)(void *ctx, uint8_t dev_addr, uint8_t reg, uint8_t *data, size_t len);
    void (*delay_ms)(void *ctx, uint32_t ms);
    void *ctx;
} mpu6050_bus_t;

/* -------------------------------------------------------------- Data types */
typedef struct {
    int16_t ax, ay, az;
    int16_t temp;
    int16_t gx, gy, gz;
} mpu6050_raw_t;

typedef struct {
    float ax_g, ay_g, az_g;          /* acceleration [g], bias-corrected   */
    float gx_dps, gy_dps, gz_dps;    /* angular rate [deg/s], bias-corrected */
    float temp_c;                    /* die temperature [degC]             */
} mpu6050_data_t;

typedef struct {
    float accel_bias_g[3];   /* subtracted from ax, ay, az */
    float gyro_bias_dps[3];  /* subtracted from gx, gy, gz */
} mpu6050_calib_t;

typedef struct {
    mpu6050_bus_t    bus;
    uint8_t          addr;
    mpu6050_config_t cfg;
    float            accel_lsb_per_g;
    float            gyro_lsb_per_dps;
    mpu6050_calib_t  calib;
    bool             initialized;
} mpu6050_t;

/* --------------------------------------------------------------------- API */
/**
 * Probe WHO_AM_I, reset the chip, wake it (PLL with X gyro clock) and apply cfg.
 * @param cfg  NULL -> MPU6050_CONFIG_DEFAULT.
 * Calibration is reset to zero bias.
 */
mpu6050_err_t mpu6050_init(mpu6050_t *dev, const mpu6050_bus_t *bus, uint8_t addr,
                           const mpu6050_config_t *cfg);

/** Burst-read the 14 measurement bytes (big-endian) into raw counts. */
mpu6050_err_t mpu6050_read_raw(mpu6050_t *dev, mpu6050_raw_t *raw);

/** Read and convert to physical units with calibration applied. */
mpu6050_err_t mpu6050_read(mpu6050_t *dev, mpu6050_data_t *out);

/** Pure conversion helper (no bus access) - used by mpu6050_read and tests. */
void mpu6050_convert(const mpu6050_t *dev, const mpu6050_raw_t *raw, mpu6050_data_t *out);

/**
 * Average `samples` readings while the vehicle is stationary and level and
 * store biases in dev->calib. Expected rest vector is (0, 0, +1 g).
 * Waits delay_ms(2) between samples.  samples == 0 -> MPU6050_ERR_ARG.
 */
mpu6050_err_t mpu6050_calibrate(mpu6050_t *dev, uint16_t samples);

/** Apply a previously stored calibration (e.g. loaded from SD/NVS). */
void mpu6050_set_calibration(mpu6050_t *dev, const mpu6050_calib_t *calib);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
