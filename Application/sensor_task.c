/**
 * @file    sensor_task.c
 * @brief   Sensor acquisition task (see sensor_task.h for the public contract).
 *
 * Single-instance module: all state is static, no dynamic allocation except the
 * FreeRTOS objects (mutex, queue, task) created once in sensor_task_start().
 */
#ifdef ESP_PLATFORM

#include "sensor_task.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "hall_port_esp32.h"
#include "mpu6050_port_esp32.h"

#define TAG "sensor_task"

/** IMU consecutive read errors before the driver is considered lost. */
#define SENSOR_IMU_MAX_ERRORS     10u
/** Minimum time between two mpu6050_init() retries. */
#define SENSOR_IMU_RETRY_MS       1000u
/** Max time to wait for the data mutex. */
#define SENSOR_MUTEX_TIMEOUT_MS   5u

/* ------------------------------------------------------------------ State */
static sensor_task_config_t s_cfg;
static bool                 s_started;

static SemaphoreHandle_t    s_mutex;
static QueueHandle_t        s_queue;
static TaskHandle_t         s_task;

/* Owned exclusively by the sensor task after start. */
static mpu6050_bus_t        s_bus;
static bool                 s_bus_ready;
static mpu6050_t            s_mpu;
static bool                 s_imu_ready;
static bool                 s_imu_calibrated;
static hall_t               s_hall;
static speed_t              s_speed;
static event_detector_t     s_detector;

/* Shared with other tasks, protected by s_mutex. */
static sensor_record_t      s_latest;
static bool                 s_has_latest;
static sensor_record_t      s_history[SENSOR_HISTORY_LEN];
static size_t               s_hist_head;    /* next write index        */
static size_t               s_hist_count;   /* valid records (<= LEN)  */

static volatile uint32_t    s_dropped_events;

/* --------------------------------------------------------------- Helpers */
/** @brief Fill cfg with the defaults documented in sensor_task.h. */
void sensor_task_default_config(sensor_task_config_t *cfg)
{
    static const mpu6050_config_t         k_mpu   = MPU6050_CONFIG_DEFAULT;
    static const hall_config_t            k_hall  = HALL_CONFIG_DEFAULT;
    static const speed_config_t           k_speed = SPEED_CONFIG_DEFAULT;
    static const event_detection_config_t k_event = EVENT_DETECTION_CONFIG_DEFAULT;

    if (cfg == NULL) {
        return;
    }
    memset(cfg, 0, sizeof(*cfg));
    cfg->i2c_port        = (int)I2C_NUM_0;
    cfg->sda_gpio        = 21;
    cfg->scl_gpio        = 22;
    cfg->i2c_clk_hz      = 400000u;
    cfg->mpu_addr        = MPU6050_ADDR_AD0_LOW;
    cfg->calib_samples   = 200u;
    cfg->hall_gpio       = 27;
    cfg->hall_active_low = true;
    cfg->mpu_cfg         = k_mpu;
    cfg->hall_cfg        = k_hall;
    cfg->speed_cfg       = k_speed;
    cfg->event_cfg       = k_event;
    cfg->task_stack      = 4096u;
    cfg->task_priority   = 5u;
    cfg->task_core       = 1;
}

/** @brief Release FreeRTOS objects after a failed start so start can be retried. */
static void sensor_cleanup(void)
{
    if (s_queue != NULL) {
        vQueueDelete(s_queue);
        s_queue = NULL;
    }
    if (s_mutex != NULL) {
        vSemaphoreDelete(s_mutex);
        s_mutex = NULL;
    }
    s_started = false;
}

/**
 * @brief (Re-)initialise the IMU, keeping a previously stored calibration.
 * @return true when the IMU is ready to be read.
 */
static bool sensor_imu_try_init(void)
{
    const mpu6050_calib_t saved = s_mpu.calib;   /* mpu6050_init() zeroes it */

    if (mpu6050_init(&s_mpu, &s_bus, s_cfg.mpu_addr, &s_cfg.mpu_cfg) != MPU6050_OK) {
        s_mpu.calib = saved;
        return false;
    }
    mpu6050_set_calibration(&s_mpu, &saved);
    return true;
}

/** @brief Store a record as latest + history entry (short critical section). */
static void sensor_publish(const sensor_record_t *rec)
{
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(SENSOR_MUTEX_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGW(TAG, "data mutex timeout, record dropped");
        return;
    }
    s_latest               = *rec;
    s_has_latest           = true;
    s_history[s_hist_head] = *rec;
    s_hist_head            = (s_hist_head + 1u) % SENSOR_HISTORY_LEN;
    if (s_hist_count < SENSOR_HISTORY_LEN) {
        s_hist_count++;
    }
    xSemaphoreGive(s_mutex);
}

/* ------------------------------------------------------------------ Task */
/** @brief Sensor task body: fixed-period IMU read, speed update, event detection. */
static void sensor_task_body(void *arg)
{
    (void)arg;

    TickType_t period_ticks = pdMS_TO_TICKS(SENSOR_PERIOD_MS);
    if (period_ticks == 0) {
        period_ticks = 1;
    }
    TickType_t last_wake = xTaskGetTickCount();

    uint32_t       cycle         = 0;
    uint32_t       imu_errors    = 0;
    uint32_t       last_retry_ms = (uint32_t)(esp_timer_get_time() / 1000);
    speed_result_t spd;
    memset(&spd, 0, sizeof(spd));

    for (;;) {
        vTaskDelayUntil(&last_wake, period_ticks);

        const int64_t  now_us = esp_timer_get_time();
        const uint32_t now_ms = (uint32_t)(now_us / 1000);

        /* --- IMU ----------------------------------------------------- */
        mpu6050_data_t imu;
        bool           imu_valid = false;
        memset(&imu, 0, sizeof(imu));

        if (s_imu_ready) {
            if (mpu6050_read(&s_mpu, &imu) == MPU6050_OK) {
                imu_valid  = true;
                imu_errors = 0;
            } else {
                imu_errors++;
                if (imu_errors >= SENSOR_IMU_MAX_ERRORS) {
                    ESP_LOGW(TAG, "IMU lost after %u read errors, will re-init",
                             (unsigned)imu_errors);
                    s_imu_ready   = false;
                    last_retry_ms = now_ms - SENSOR_IMU_RETRY_MS;   /* retry right away */
                }
            }
        }
        if (!s_imu_ready && s_bus_ready &&
            (uint32_t)(now_ms - last_retry_ms) >= SENSOR_IMU_RETRY_MS) {
            last_retry_ms = now_ms;
            if (sensor_imu_try_init()) {
                ESP_LOGI(TAG, "IMU (re)initialised");
                s_imu_ready = true;
                imu_errors  = 0;
            }
        }

        /* --- Speed (every SENSOR_SPEED_DIV cycles) ---------------------- */
        if ((cycle % SENSOR_SPEED_DIV) == 0u) {
            hall_snapshot_t snap;
            hall_get_snapshot(&s_hall, &snap);
            speed_update(&s_speed, &snap, (uint64_t)now_us, &spd);
        }
        cycle++;

        /* --- Record ---------------------------------------------------- */
        sensor_record_t rec;
        memset(&rec, 0, sizeof(rec));
        rec.timestamp_ms = now_ms;
        rec.speed_kmh    = spd.speed_kmh;
        rec.distance_m   = spd.distance_m;
        if (imu_valid) {
            rec.ax_g    = imu.ax_g;
            rec.ay_g    = imu.ay_g;
            rec.az_g    = imu.az_g;
            rec.gx_dps  = imu.gx_dps;
            rec.gy_dps  = imu.gy_dps;
            rec.gz_dps  = imu.gz_dps;
            rec.temp_c  = imu.temp_c;
            rec.status |= SENSOR_STATUS_IMU_OK;
        }
        if (s_imu_calibrated) {
            rec.status |= SENSOR_STATUS_IMU_CAL;
        }

        /* --- Event detection (valid IMU samples only) -------------------- */
        event_t evt;
        bool    fired = false;
        memset(&evt, 0, sizeof(evt));
        if (imu_valid) {
            event_sample_t es;
            es.timestamp_ms = now_ms;
            es.ax_g         = imu.ax_g;
            es.ay_g         = imu.ay_g;
            es.az_g         = imu.az_g;
            es.gx_dps       = imu.gx_dps;
            es.gy_dps       = imu.gy_dps;
            es.gz_dps       = imu.gz_dps;
            es.speed_kmh    = spd.speed_kmh;
            fired = event_detection_process(&s_detector, &es, &evt);
        }

        sensor_publish(&rec);

        if (fired) {
            ESP_LOGI(TAG, "EVENT %s: peak=%.2f g, speed=%.1f km/h, severity=%.2f",
                     event_type_str(evt.type), (double)evt.peak_g,
                     (double)evt.speed_kmh, (double)evt.severity);
            if (xQueueSend(s_queue, &evt, 0) != pdTRUE) {
                s_dropped_events++;
                ESP_LOGW(TAG, "event queue full, event dropped");
            }
        }
    }
}

/* ---------------------------------------------------------------- Public */
/** @brief Initialise all modules and start the sensor task. */
int sensor_task_start(const sensor_task_config_t *cfg)
{
    if (cfg == NULL) {
        return -1;
    }
    if (s_started) {
        ESP_LOGW(TAG, "already started");
        return -2;
    }
    s_cfg     = *cfg;
    s_started = true;

    s_has_latest     = false;
    s_hist_head      = 0;
    s_hist_count     = 0;
    s_dropped_events = 0;
    s_imu_ready      = false;
    s_imu_calibrated = false;
    s_bus_ready      = false;

    s_mutex = xSemaphoreCreateMutex();
    s_queue = xQueueCreate(SENSOR_EVENT_QUEUE_LEN, sizeof(event_t));
    if (s_mutex == NULL || s_queue == NULL) {
        ESP_LOGE(TAG, "mutex/queue creation failed");
        sensor_cleanup();
        return -3;
    }

    /* Algorithms first: pure computation, only fails on an invalid config. */
    if (speed_init(&s_speed, &s_cfg.speed_cfg) != 0) {
        ESP_LOGE(TAG, "invalid speed config");
        sensor_cleanup();
        return -4;
    }
    if (event_detection_init(&s_detector, &s_cfg.event_cfg) != 0) {
        ESP_LOGE(TAG, "invalid event detection config");
        sensor_cleanup();
        return -5;
    }

    /* Hall first: fatal on error. Done before the I2C port because that port has a
     * single static context that cannot be re-created after a failed start. */
    hall_init(&s_hall, &s_cfg.hall_cfg);
    esp_err_t err = hall_port_esp32_init(&s_hall, (gpio_num_t)s_cfg.hall_gpio, s_cfg.hall_active_low);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Hall GPIO init failed (%s)", esp_err_to_name(err));
        sensor_cleanup();
        return -6;
    }

    /* IMU: failure is not fatal, the task retries later. */
    err = mpu6050_port_esp32_init(&s_bus, (i2c_port_num_t)s_cfg.i2c_port,
                                            (gpio_num_t)s_cfg.sda_gpio,
                                            (gpio_num_t)s_cfg.scl_gpio,
                                            s_cfg.mpu_addr, s_cfg.i2c_clk_hz);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "I2C port init failed (%s), IMU disabled", esp_err_to_name(err));
    } else {
        s_bus_ready = true;
        if (mpu6050_init(&s_mpu, &s_bus, s_cfg.mpu_addr, &s_cfg.mpu_cfg) != MPU6050_OK) {
            ESP_LOGW(TAG, "MPU6050 not found / init failed, will retry");
        } else {
            s_imu_ready = true;
            if (s_cfg.calib_samples > 0) {
                ESP_LOGI(TAG, "calibrating IMU (%u samples), keep the vehicle still",
                         (unsigned)s_cfg.calib_samples);
                if (mpu6050_calibrate(&s_mpu, s_cfg.calib_samples) == MPU6050_OK) {
                    s_imu_calibrated = true;
                } else {
                    ESP_LOGW(TAG, "IMU calibration failed, using zero bias");
                }
            }
        }
    }

    /* Restart the trip distance here so pulses during calibration are not counted. */
    hall_snapshot_t snap;
    hall_get_snapshot(&s_hall, &snap);
    speed_reset_distance(&s_speed, snap.pulse_count);

    /* A core index the chip does not have (e.g. single-core targets) -> no affinity. */
    BaseType_t core = (s_cfg.task_core >= 0 && s_cfg.task_core < (int)portNUM_PROCESSORS)
                          ? (BaseType_t)s_cfg.task_core
                          : tskNO_AFFINITY;
    if (xTaskCreatePinnedToCore(sensor_task_body, "sensor", s_cfg.task_stack, NULL,
                                (UBaseType_t)s_cfg.task_priority, &s_task, core) != pdPASS) {
        ESP_LOGE(TAG, "task creation failed");
        sensor_cleanup();
        return -7;
    }

    ESP_LOGI(TAG, "started (%u ms period, IMU %s)", (unsigned)SENSOR_PERIOD_MS,
             s_imu_ready ? "ok" : "unavailable");
    return 0;
}

/** @brief Copy the newest record. */
bool sensor_get_latest(sensor_record_t *out)
{
    if (out == NULL || s_mutex == NULL) {
        return false;
    }
    bool ok = false;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(SENSOR_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        if (s_has_latest) {
            *out = s_latest;
            ok   = true;
        }
        xSemaphoreGive(s_mutex);
    }
    return ok;
}

/** @brief Event queue handle (QueueHandle_t of event_t), NULL before start. */
void *sensor_get_event_queue(void)
{
    return (void *)s_queue;
}

/** @brief Copy up to `max` most recent records, oldest first. */
size_t sensor_get_history(sensor_record_t *buf, size_t max)
{
    if (buf == NULL || max == 0 || s_mutex == NULL) {
        return 0;
    }
    size_t n = 0;
    if (xSemaphoreTake(s_mutex, pdMS_TO_TICKS(SENSOR_MUTEX_TIMEOUT_MS)) == pdTRUE) {
        n = (s_hist_count < max) ? s_hist_count : max;
        size_t idx = (s_hist_head + SENSOR_HISTORY_LEN - n) % SENSOR_HISTORY_LEN;
        for (size_t i = 0; i < n; i++) {
            buf[i] = s_history[idx];
            idx    = (idx + 1u) % SENSOR_HISTORY_LEN;
        }
        xSemaphoreGive(s_mutex);
    }
    return n;
}

/** @brief Number of events dropped because the queue was full. */
uint32_t sensor_get_dropped_events(void)
{
    return s_dropped_events;
}

#endif /* ESP_PLATFORM */
