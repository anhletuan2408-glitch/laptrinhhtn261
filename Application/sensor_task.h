/**
 * @file    sensor_task.h
 * @brief   Sensor acquisition task (ESP32 / FreeRTOS): MPU6050 + Hall + speed + event detection.
 *
 * Public interface for the other modules (Logger/SD, OLED HMI, system task):
 *   - sensor_get_latest()     : newest fused sample (thread-safe copy)
 *   - sensor_get_event_queue(): FreeRTOS queue of event_t (HARD_BRAKE / CRASH)
 *   - sensor_get_history()    : last SENSOR_HISTORY_LEN samples (pre-event data for the black box)
 *
 * Timing: the task runs every SENSOR_PERIOD_MS (vTaskDelayUntil). Each cycle it
 * reads the IMU, updates the speed every SENSOR_SPEED_DIV cycles, runs the
 * event detector, stores the record and pushes any event (non-blocking).
 */
#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "event_detection.h"
#include "hall.h"
#include "mpu6050.h"
#include "speed.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SENSOR_PERIOD_MS      10u    /* 100 Hz IMU sampling                  */
#define SENSOR_SPEED_DIV      5u     /* speed update every 5 cycles = 20 Hz  */
#define SENSOR_HISTORY_LEN    200u   /* 2 s of history at 100 Hz             */
#define SENSOR_EVENT_QUEUE_LEN 8u

/** Status bits in sensor_record_t.status */
#define SENSOR_STATUS_IMU_OK   0x01u   /* IMU sample valid this cycle           */
#define SENSOR_STATUS_IMU_CAL  0x02u   /* IMU calibrated                        */

typedef struct {
    uint32_t timestamp_ms;      /* ms since boot (esp_timer_get_time()/1000) */
    float    ax_g, ay_g, az_g;
    float    gx_dps, gy_dps, gz_dps;
    float    temp_c;
    float    speed_kmh;
    float    distance_m;
    uint8_t  status;            /* SENSOR_STATUS_* */
} sensor_record_t;

typedef struct {
    /* MPU6050 / I2C */
    int      i2c_port;          /* I2C_NUM_0                    */
    int      sda_gpio;          /* 21                           */
    int      scl_gpio;          /* 22                           */
    uint32_t i2c_clk_hz;        /* 400000                       */
    uint8_t  mpu_addr;          /* MPU6050_ADDR_AD0_LOW         */
    uint16_t calib_samples;     /* 200 (0 = skip calibration)   */
    /* Hall */
    int      hall_gpio;         /* 27                           */
    bool     hall_active_low;   /* true (A3144 / KY-003)        */
    /* Algorithms */
    mpu6050_config_t         mpu_cfg;
    hall_config_t            hall_cfg;
    speed_config_t           speed_cfg;
    event_detection_config_t event_cfg;
    /* Task */
    uint32_t task_stack;        /* 4096                         */
    uint8_t  task_priority;     /* 5                            */
    int      task_core;         /* 1 (APP CPU)                  */
} sensor_task_config_t;

/** Fill cfg with the defaults listed in the comments above. */
void sensor_task_default_config(sensor_task_config_t *cfg);

/**
 * Initialise I2C, MPU6050 (+ optional calibration, vehicle must be still and level),
 * Hall GPIO interrupt, speed and event detector, then start the task.
 * If the IMU is not found the task still starts (speed keeps working) and retries
 * mpu6050_init() every 1 s; records have SENSOR_STATUS_IMU_OK cleared.
 * @return 0 on success, negative on fatal error (queue/mutex/task creation, Hall GPIO).
 */
int sensor_task_start(const sensor_task_config_t *cfg);

/** Copy the newest record. Returns false if no record has been produced yet. */
bool sensor_get_latest(sensor_record_t *out);

/** Queue of event_t (opaque QueueHandle_t to avoid FreeRTOS includes here). NULL before start. */
void *sensor_get_event_queue(void);

/**
 * Copy up to `max` most recent records, oldest first, into buf.
 * @return number of records copied.
 */
size_t sensor_get_history(sensor_record_t *buf, size_t max);

/** Number of events dropped because the queue was full (diagnostics). */
uint32_t sensor_get_dropped_events(void);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_TASK_H */
