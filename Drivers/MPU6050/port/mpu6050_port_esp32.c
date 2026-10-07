/**
 * @file    mpu6050_port_esp32.c
 * @brief   ESP-IDF v5.2+ I2C master port for the MPU6050 driver.
 */
#ifdef ESP_PLATFORM

#include "mpu6050_port_esp32.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define PORT_TIMEOUT_MS     50
#define PORT_MAX_WRITE_LEN  16u

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint8_t                 addr;
    bool                    ready;
} port_ctx_t;

static port_ctx_t s_ctx;

/**
 * @brief Write `len` bytes starting at register `reg` (single I2C transaction).
 */
static int port_write(void *ctx, uint8_t dev_addr, uint8_t reg, const uint8_t *data, size_t len)
{
    port_ctx_t *p = (port_ctx_t *)ctx;
    uint8_t buf[1u + PORT_MAX_WRITE_LEN];

    if (p == NULL || !p->ready || dev_addr != p->addr ||
        (len > 0u && data == NULL) || len > PORT_MAX_WRITE_LEN) {
        return -1;
    }
    buf[0] = reg;
    if (len > 0u) {
        memcpy(&buf[1], data, len);
    }
    return (i2c_master_transmit(p->dev, buf, 1u + len, PORT_TIMEOUT_MS) == ESP_OK) ? 0 : -1;
}

/**
 * @brief Read `len` bytes starting at register `reg` (repeated-start transaction).
 */
static int port_read(void *ctx, uint8_t dev_addr, uint8_t reg, uint8_t *data, size_t len)
{
    port_ctx_t *p = (port_ctx_t *)ctx;

    if (p == NULL || !p->ready || dev_addr != p->addr || data == NULL || len == 0u) {
        return -1;
    }
    return (i2c_master_transmit_receive(p->dev, &reg, 1u, data, len, PORT_TIMEOUT_MS) == ESP_OK)
               ? 0 : -1;
}

/**
 * @brief Block the calling task for at least `ms` milliseconds (minimum 1 tick).
 */
static void port_delay_ms(void *ctx, uint32_t ms)
{
    TickType_t ticks = pdMS_TO_TICKS(ms);

    (void)ctx;
    if (ticks == 0) {
        ticks = 1;
    }
    vTaskDelay(ticks);
}

/**
 * @brief Create the I2C bus, add the MPU6050 device and fill the bus descriptor.
 */
esp_err_t mpu6050_port_esp32_init(mpu6050_bus_t *bus, i2c_port_num_t port,
                                  gpio_num_t sda, gpio_num_t scl,
                                  uint8_t addr, uint32_t clk_hz)
{
    esp_err_t err;

    if (bus == NULL || clk_hz == 0u || addr > 0x7Fu) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ctx.ready) {
        return ESP_ERR_INVALID_STATE;
    }

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = port,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    err = i2c_new_master_bus(&bus_cfg, &s_ctx.bus);
    if (err != ESP_OK) {
        return err;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = clk_hz,
    };
    err = i2c_master_bus_add_device(s_ctx.bus, &dev_cfg, &s_ctx.dev);
    if (err != ESP_OK) {
        (void)i2c_del_master_bus(s_ctx.bus);
        s_ctx.bus = NULL;
        return err;
    }

    s_ctx.addr = addr;
    s_ctx.ready = true;

    bus->write = port_write;
    bus->read = port_read;
    bus->delay_ms = port_delay_ms;
    bus->ctx = &s_ctx;
    return ESP_OK;
}

#endif /* ESP_PLATFORM */
